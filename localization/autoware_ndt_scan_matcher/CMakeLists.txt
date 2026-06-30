cmake_minimum_required(VERSION 3.14)
project(autoware_ndt_scan_matcher)

# cspell:ignore multigrid

find_package(autoware_cmake REQUIRED)
autoware_package()

# Compile flags for SIMD instructions
# Be careful to change these options, especially when `ndt_omp` implementation is used.
# All packages linked to `ndt_omp` should use the same SIMD instruction set.
# In case mismatched instruction set are used, program causes a crash at its initialization
# because of a misaligned access to the `Eigen` libraries' data structure.
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  # For x86_64 architecture, SIMD instruction set is fixed below versions,
  # because the `ndt_omp` is optimized to these versions.
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
else()
  # For other architecture, like arm64, compile flags are generally prepared by compiler
  # march=native is disabled as default for specific depending pcl libraries
  # or pre-building packages for other computers.
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()

find_package(OpenMP)

find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

# Rust port build switch (see plan/ndt_in_rust.md, 案B). When OFF (default) the package builds the
# original C++ unchanged and does not depend on Rust/Corrosion at all. When ON, the nested
# `autoware_ndt_scan_matcher_rs` crate is built via Corrosion, linked over the C ABI, and the node's
# callbacks dispatch their compute to the Rust engine (the per-callback `#ifdef NDT_USE_RUST` in the
# .cpp). The C++ sources are the same in both configs — the only engine-type switch is the typedef
# in ndt_backend.hpp. Defined before the libraries so both `multigrid_ndt_omp` and ${PROJECT_NAME}
# can use it.
option(NDT_USE_RUST "Build/use the Rust port (autoware_ndt_scan_matcher_rs) over FFI" OFF)

if(NDT_USE_RUST)
  # Corrosion is fetched at configure time; for offline builds switch GIT_* to a local SOURCE_DIR.
  include(FetchContent)
  FetchContent_Declare(Corrosion
    GIT_REPOSITORY https://github.com/corrosion-rs/corrosion.git
    GIT_TAG v0.5
  )
  FetchContent_MakeAvailable(Corrosion)
  # `ros` feature builds the bindgen ROS-message bindings (geometry_msgs Pose) used by the
  # zero-copy FFI shims. bindgen needs the rosidl C headers; pass geometry_msgs' include dir.
  corrosion_import_crate(MANIFEST_PATH
    ${CMAKE_CURRENT_SOURCE_DIR}/autoware_ndt_scan_matcher_rs/Cargo.toml
    FEATURES ros
  )
  find_package(geometry_msgs REQUIRED)
  string(JOIN ":" NDT_ROS_INCLUDE_DIRS ${geometry_msgs_INCLUDE_DIRS})
  corrosion_set_env_vars(autoware_ndt_scan_matcher_rs
    "ROS_INCLUDE_DIRS=${NDT_ROS_INCLUDE_DIRS}")
  # Generate the C FFI header (autoware_ndt_scan_matcher_rs.h) from the Rust `#[repr(C)]` structs +
  # `extern "C"` fns via cbindgen (config: autoware_ndt_scan_matcher_rs/cbindgen.toml). Single source
  # of truth — no hand-syncing the C++ side against the Rust FFI. Corrosion attaches the generated
  # header's include dir to the `autoware_ndt_scan_matcher_rs` target, so every target that links the
  # crate sees it transitively. (cbindgen is auto-installed by Corrosion if not on PATH.)
  corrosion_experimental_cbindgen(
    TARGET autoware_ndt_scan_matcher_rs
    HEADER_NAME autoware_ndt_scan_matcher_rs.h)
endif()

ament_auto_add_library(multigrid_ndt_omp SHARED
  src/ndt_omp/multi_voxel_grid_covariance_omp.cpp
  src/ndt_omp/multigrid_ndt_omp.cpp
  src/ndt_omp/estimate_covariance.cpp
  src/ndt_omp/estimate_covariance_math.cpp
)
target_link_libraries(multigrid_ndt_omp ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(multigrid_ndt_omp OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

if(NDT_USE_RUST)
  target_link_libraries(multigrid_ndt_omp autoware_ndt_scan_matcher_rs)
endif()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_update_module.cpp
  src/ndt_scan_matcher_core.cpp
  src/ndt_scan_matcher_helper.cpp
  src/particle.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} multigrid_ndt_omp)

if(NDT_USE_RUST)
  target_link_libraries(${PROJECT_NAME} autoware_ndt_scan_matcher_rs)
  # Drive the conditional NDT typedef in the node headers (core / map_update) to the Rust adapter.
  # PUBLIC so the generated node executable and the launch/sequence tests that link this lib also
  # compile against the adapter.
  target_compile_definitions(${PROJECT_NAME} PUBLIC NDT_USE_RUST)
endif()

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ndt_scan_matcher::NDTScanMatcher"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

if(BUILD_TESTING)
  add_launch_test(
    test/test_ndt_scan_matcher_launch.py
    TIMEOUT "30"
  )

  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(standard_sequence_for_initial_pose_estimation
    test/test_cases/standard_sequence_for_initial_pose_estimation.cpp
    TIMEOUT "300"
  )
  ament_auto_add_gtest(once_initialize_at_out_of_map_then_initialize_correctly
    test/test_cases/once_initialize_at_out_of_map_then_initialize_correctly.cpp
    TIMEOUT "300"
  )
  ament_auto_add_gtest(particles_num_less_than_publish_num
    test/test_cases/particles_num_less_than_publish_num.cpp
    TIMEOUT "300"
  )

  # Fast, dependency-light unit tests for the pure covariance / oscillation math.
  ament_auto_add_gtest(test_estimate_covariance
    test/test_estimate_covariance.cpp
  )
  target_link_libraries(test_estimate_covariance multigrid_ndt_omp ${PCL_LIBRARIES})

  ament_auto_add_gtest(test_ndt_scan_matcher_helper
    test/test_ndt_scan_matcher_helper.cpp
  )
  target_link_libraries(test_ndt_scan_matcher_helper ${PROJECT_NAME})

  if(NDT_USE_RUST)
    # Rust port tests (only relevant when the Rust crate is built).
    # 1. Run the Rust crate's own unit tests through CTest (so `colcon test` / `./test.sh`
    #    pick them up). cargo's test artifacts go under the CMake build dir, not the source tree.
    find_program(CARGO_EXECUTABLE cargo REQUIRED)
    add_test(
      NAME autoware_ndt_scan_matcher_rs_cargo_test
      COMMAND ${CARGO_EXECUTABLE} test --features ros
              --manifest-path ${CMAKE_CURRENT_SOURCE_DIR}/autoware_ndt_scan_matcher_rs/Cargo.toml
              --target-dir ${CMAKE_CURRENT_BINARY_DIR}/rs_test_target
    )
    # bindgen (run during the test build) needs the rosidl C headers.
    set_tests_properties(autoware_ndt_scan_matcher_rs_cargo_test PROPERTIES
      ENVIRONMENT "ROS_INCLUDE_DIRS=${NDT_ROS_INCLUDE_DIRS}")

    # 2. Differential test: Rust voxel_grid vs the C++ MultiVoxelGridCovariance (engine port E2).
    ament_auto_add_gtest(test_voxel_grid
      test/test_voxel_grid.cpp
    )
    target_link_libraries(test_voxel_grid
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 3. Differential test: Rust align (FFI) vs the C++ NDT engine (engine port E4d).
    ament_auto_add_gtest(test_align
      test/test_align.cpp
    )
    target_link_libraries(test_align
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 4. Differential test: Rust multi-NDT covariance estimation (FFI) vs C++ pclomp (engine port E5).
    ament_auto_add_gtest(test_estimate_covariance_multi
      test/test_estimate_covariance_multi.cpp
    )
    target_link_libraries(test_estimate_covariance_multi
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 5. Differential test: Rust persistent engine handle (FFI) vs the C++ NDT engine (E6a).
    ament_auto_add_gtest(test_ndt_engine
      test/test_ndt_engine.cpp
    )
    target_link_libraries(test_ndt_engine
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 6. Differential test: the drop-in NdtRustAdapter vs the C++ NDT engine (engine port E6b).
    ament_auto_add_gtest(test_ndt_rust_adapter
      test/test_ndt_rust_adapter.cpp
    )
    target_link_libraries(test_ndt_rust_adapter
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 7. Differential test: the convergence decision migrated to Rust (FFI) vs the C++ gate logic
    #    from callback_sensor_points_main (node port N1).
    ament_auto_add_gtest(test_convergence_verdict
      test/test_convergence_verdict.cpp
    )
    target_link_libraries(test_convergence_verdict autoware_ndt_scan_matcher_rs)

    # 8. Test: the thin pose callbacks migrated to Rust (FFI) drive a mock host vtable (node port N2).
    ament_auto_add_gtest(test_node_pose_callbacks
      test/test_node_pose_callbacks.cpp
    )
    target_link_libraries(test_node_pose_callbacks autoware_ndt_scan_matcher_rs)

    # 9. Differential test: the map-update distance decision migrated to Rust (FFI) vs the C++
    #    MapUpdateModule should_update_map / out_of_map_range math (node port N3).
    ament_auto_add_gtest(test_map_update_verdict
      test/test_map_update_verdict.cpp
    )
    target_link_libraries(test_map_update_verdict autoware_ndt_scan_matcher_rs)

    # 10. Test: the sensor-callback align orchestrator (run_align, FFI) drives the live engine handle
    #     and matches the adapter's own align (node port N4a).
    ament_auto_add_gtest(test_node_run_align
      test/test_node_run_align.cpp
    )
    target_link_libraries(test_node_run_align
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 11. Test: the covariance orchestrator (FFI) vs a hand-composition on the same adapter (N4b).
    ament_auto_add_gtest(test_estimate_pose_covariance
      test/test_estimate_pose_covariance.cpp
    )
    target_link_libraries(test_estimate_pose_covariance
      multigrid_ndt_omp ${PCL_LIBRARIES} autoware_ndt_scan_matcher_rs)

    # 12. Test: the opaque NdtScanMatcherRs node handle + C++ RAII wrapper (roadmap foundation).
    ament_auto_add_gtest(test_ndt_scan_matcher_rs_handle
      test/test_ndt_scan_matcher_rs_handle.cpp
    )
    target_link_libraries(test_ndt_scan_matcher_rs_handle
      ${PROJECT_NAME} autoware_ndt_scan_matcher_rs)

    # 13. Differential test: the Rust-owned regularization pose buffer (FFI) vs the C++ SmartPoseBuffer
    #     interpolation (node port: Phase 1 slice A).
    ament_auto_add_gtest(test_regularization_buffer
      test/test_regularization_buffer.cpp
    )
    target_link_libraries(test_regularization_buffer
      ${PROJECT_NAME} autoware_ndt_scan_matcher_rs)

    # 14. Differential test: the Rust-owned initial-pose buffer (FFI) vs the C++ SmartPoseBuffer +
    #     activation gating (node port: Phase 1 slice B).
    ament_auto_add_gtest(test_initial_pose_buffer
      test/test_initial_pose_buffer.cpp
    )
    target_link_libraries(test_initial_pose_buffer
      ${PROJECT_NAME} autoware_ndt_scan_matcher_rs)

    # 15. Test: the Rust-owned map-update decision state machine (FFI) — node port Phase 6.
    ament_auto_add_gtest(test_map_update_state
      test/test_map_update_state.cpp
    )
    target_link_libraries(test_map_update_state
      ${PROJECT_NAME} autoware_ndt_scan_matcher_rs)
  endif()
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
