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

ament_auto_add_library(multigrid_ndt_omp SHARED
  src/ndt_omp/multi_voxel_grid_covariance_omp.cpp
  src/ndt_omp/multigrid_ndt_omp.cpp
)
target_link_libraries(multigrid_ndt_omp ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(multigrid_ndt_omp OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

# NO_TARGET_LINK_LIBRARIES: the node does not use ndt_omp. Only the backend "ndt_omp" does.
ament_auto_add_library(${PROJECT_NAME} SHARED NO_TARGET_LINK_LIBRARIES
  src/map_update_module.cpp
  src/ndt_backend_loader.cpp
  src/ndt_omp/estimate_covariance.cpp
  src/ndt_scan_matcher_core.cpp
  src/ndt_scan_matcher_helper.cpp
  src/particle.cpp
)

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ndt_scan_matcher::NDTScanMatcher"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

# The NDT backend "ndt_omp". The node loads it with pluginlib, so no other library may link it. It
# is a plain library: ament_auto exports every library that ament_auto_add_library() adds, and it
# links them into the later libraries and tests of this package and into the packages that depend on
# it.
add_library(${PROJECT_NAME}_omp_backend SHARED
  src/omp_ndt_backend.cpp
)
target_link_libraries(${PROJECT_NAME}_omp_backend multigrid_ndt_omp ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_omp_backend autoware_ndt_scan_matcher_common pluginlib)
install(TARGETS ${PROJECT_NAME}_omp_backend
  LIBRARY DESTINATION lib
)
pluginlib_export_plugin_description_file(autoware_ndt_scan_matcher_common plugins.xml)

if(BUILD_TESTING)
  # Three test backends in an install prefix inside the build directory, for the tests of the
  # error messages. pluginlib finds them only in a test that has this prefix in AMENT_PREFIX_PATH.
  set(test_backend_prefix ${CMAKE_CURRENT_BINARY_DIR}/test_ndt_backend_prefix)
  set(test_backend_index ${test_backend_prefix}/share/ament_index/resource_index)
  file(WRITE ${test_backend_index}/packages/test_ndt_backend "")
  file(WRITE
    ${test_backend_index}/autoware_ndt_scan_matcher_common__pluginlib__plugin/test_ndt_backend
    "share/test_ndt_backend/test_ndt_backend_plugins.xml\n")
  file(WRITE ${test_backend_prefix}/share/test_ndt_backend/package.xml
    "<package format=\"3\"><name>test_ndt_backend</name></package>\n")
  # configure_file() copies the file again when it changes.
  # cspell: ignore COPYONLY
  configure_file(test/test_ndt_backend_plugins.xml
    ${test_backend_prefix}/share/test_ndt_backend/test_ndt_backend_plugins.xml COPYONLY)
  foreach(variant constructor_throws initialize_throws other_abi)
    add_library(test_ndt_backend_${variant} MODULE test/test_ndt_backend.cpp)
    ament_target_dependencies(test_ndt_backend_${variant}
      autoware_ndt_scan_matcher_common pluginlib)
    # $<1:...> keeps a multi-config generator from adding a directory for each configuration.
    set_target_properties(test_ndt_backend_${variant} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY $<1:${test_backend_prefix}/lib>)
  endforeach()
  target_compile_definitions(test_ndt_backend_constructor_throws PRIVATE
    TEST_NDT_BACKEND_CONSTRUCTOR_THROWS)
  target_compile_definitions(test_ndt_backend_initialize_throws PRIVATE
    TEST_NDT_BACKEND_INITIALIZE_THROWS)
  # The other heap allocator of Eigen. test_ndt_backend_loader skips its test in a build where
  # this definition does not change the allocator.
  target_compile_definitions(test_ndt_backend_other_abi PRIVATE
    EIGEN_MALLOC_ALREADY_ALIGNED=0)

  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    add_launch_test(
      test/test_ndt_scan_matcher_launch.py
      TIMEOUT "30"
    )

    find_package(ament_cmake_gtest REQUIRED)
    find_package(ament_cmake_ros REQUIRED)

    ament_add_ros_isolated_gtest(standard_sequence_for_initial_pose_estimation
      test/test_cases/standard_sequence_for_initial_pose_estimation.cpp
      TIMEOUT "300"
    )
    target_link_libraries(standard_sequence_for_initial_pose_estimation ${PROJECT_NAME})
    ament_target_dependencies(standard_sequence_for_initial_pose_estimation
      ament_index_cpp rcl_yaml_param_parser)

    ament_add_ros_isolated_gtest(once_initialize_at_out_of_map_then_initialize_correctly
      test/test_cases/once_initialize_at_out_of_map_then_initialize_correctly.cpp
      TIMEOUT "300"
    )
    target_link_libraries(once_initialize_at_out_of_map_then_initialize_correctly ${PROJECT_NAME})
    ament_target_dependencies(once_initialize_at_out_of_map_then_initialize_correctly
      ament_index_cpp rcl_yaml_param_parser)

    ament_add_ros_isolated_gtest(particles_num_less_than_publish_num
      test/test_cases/particles_num_less_than_publish_num.cpp
      TIMEOUT "300"
    )
    target_link_libraries(particles_num_less_than_publish_num ${PROJECT_NAME})
    ament_target_dependencies(particles_num_less_than_publish_num
      ament_index_cpp rcl_yaml_param_parser)

    # Characterization tests: they pin the node's current observable behaviour through
    # `/diagnostics`, its output topics and its services, so that extracting the decision logic into
    # a ROS-free core can be shown to preserve it.
    ament_add_ros_isolated_gtest(test_ndt_scan_matcher_characteristics
      test/test_ndt_scan_matcher_characteristics.cpp
      TIMEOUT "600"
      APPEND_ENV AMENT_PREFIX_PATH=${test_backend_prefix}
    )
    add_dependencies(test_ndt_scan_matcher_characteristics test_ndt_backend_initialize_throws)
    target_link_libraries(test_ndt_scan_matcher_characteristics ${PROJECT_NAME})
    ament_target_dependencies(test_ndt_scan_matcher_characteristics
      ament_index_cpp rcl_yaml_param_parser)
  endif()

  # Fast, dependency-light unit tests for the pure covariance / oscillation math.
  ament_auto_add_gtest(test_estimate_covariance
    test/test_estimate_covariance.cpp
  )
  target_link_libraries(test_estimate_covariance ${PROJECT_NAME} ${PCL_LIBRARIES})

  ament_auto_add_gtest(test_ndt_scan_matcher_helper
    test/test_ndt_scan_matcher_helper.cpp
  )
  target_link_libraries(test_ndt_scan_matcher_helper ${PROJECT_NAME})

  # Unit tests for the map update module driven by an injected fake pcd loader (no ROS service).
  ament_auto_add_gtest(test_map_update_module
    test/test_map_update_module.cpp
  )
  target_link_libraries(test_map_update_module ${PROJECT_NAME} ${PCL_LIBRARIES})

  # The OMP backend against ndt_omp, and its copies.
  ament_auto_add_gtest(test_omp_ndt_backend
    test/test_omp_ndt_backend.cpp
  )
  target_include_directories(test_omp_ndt_backend PRIVATE src)
  target_link_libraries(test_omp_ndt_backend ${PROJECT_NAME}_omp_backend ${PCL_LIBRARIES})

  # pluginlib loads the backend library here. The test does not link it.
  ament_auto_add_gtest(test_ndt_backend_loader
    test/test_ndt_backend_loader.cpp
    APPEND_ENV AMENT_PREFIX_PATH=${test_backend_prefix}
  )
  target_link_libraries(test_ndt_backend_loader ${PROJECT_NAME} ${CMAKE_DL_LIBS})
  add_dependencies(test_ndt_backend_loader
    test_ndt_backend_constructor_throws test_ndt_backend_other_abi)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
