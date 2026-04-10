cmake_minimum_required(VERSION 3.14)

project(autoware_trajectory)

# cspell: ignore Warray, Wstringop, stringop
# Disable array bounds warnings for GCC 13+ as they appear to be false positives
# NOTE: workaround https://github.com/autowarefoundation/autoware_core/pull/644
# Delete in the future
if(((CMAKE_CXX_COMPILER_ID MATCHES "GNU") AND NOT MINGW AND NOT HAIKU) AND CMAKE_CXX_COMPILER_VERSION GREATER_EQUAL 13)
  message(STATUS "GCC13 or later detected, disabling \"-Warray-bounds and -Wstringop-overflow\" "
    "for interpolator/lane_ids_interpolator.cpp as it appears to be a false positive")
  set_source_files_properties(src/interpolator/lane_ids_interpolator.cpp PROPERTIES
      COMPILE_FLAGS "-Wno-array-bounds -Wno-stringop-overflow"
  )
endif()

find_package(autoware_cmake REQUIRED)
autoware_package()

set(fmt_TARGETS fmt::fmt)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/detail/logging.cpp
  src/detail/time_distance_mapping.cpp
  src/detail/types.cpp
  src/detail/util.cpp
  src/interpolator/akima_spline.cpp
  src/interpolator/cubic_spline.cpp
  src/interpolator/lane_ids_interpolator.cpp
  src/interpolator/linear.cpp
  src/interpolator/result.cpp
  src/interpolator/spherical_linear.cpp
  src/path_point.cpp
  src/path_point_with_lane_id.cpp
  src/point.cpp
  src/pose.cpp
  src/temporal_trajectory.cpp
  src/threshold.cpp
  src/trajectory_point.cpp
  src/utils/closest.cpp
  src/utils/crossed.cpp
  src/utils/find_if.cpp
  src/utils/find_intervals.cpp
  src/utils/find_nearest.cpp
  src/utils/footprint.cpp
  src/utils/pretty_build.cpp
  src/utils/reference_path.cpp
  src/utils/shift.cpp
  src/utils/temporal/find_stop_points.cpp
  src/utils/velocity.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  find_package(
    Python3
    COMPONENTS Interpreter Development
    REQUIRED)
  list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS
    Python3
  )
  set(Python3_TARGETS Python3::Python)
  set(yaml-cpp_TARGETS yaml-cpp::yaml-cpp)

  set(test_files
    test/test_main.cpp
    test/test_build.cpp
    test/test_crossed.cpp
    test/test_shift.cpp
    test/test_helpers.cpp
    test/test_interpolator.cpp
    test/test_pretty_build.cpp
    test/test_reference_path.cpp
    test/test_reference_path_vm_10_spec.cpp
    test/test_reference_path_vm_10_spec_invalid.cpp
    test/test_trajectory_container.cpp
    test/test_trajectory_container_trajectory_point.cpp
    test/test_utils_find_nearest.cpp
    test/test_utils_lateral_metrics.cpp
    test/test_utils_footprint.cpp
    test/test_utils_velocity.cpp
    test/test_add_offset.cpp
  )

  ament_auto_add_gtest(test_${PROJECT_NAME} ${test_files})

  target_compile_definitions(test_${PROJECT_NAME}
    PUBLIC PLOT
  )

  set(example_files
    examples/example_find_if.cpp
    examples/example_find_intervals.cpp
    examples/example_interpolator.cpp
    examples/example_path_point.cpp
    examples/example_point.cpp
    examples/example_pose.cpp
    examples/example_pretty_build.cpp
    examples/example_readme.cpp
    examples/example_reference_path.cpp
    examples/example_shift.cpp
    examples/example_self_intersecting.cpp
    examples/example_lateral_metrics.cpp
    examples/example_footprint.cpp
    examples/example_crossed.cpp
    examples/example_add_offset.cpp
    examples/example_temporal_trajectory.cpp
    examples/example_temporal_crop.cpp
    examples/example_add_offset.cpp
  )

  foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    ament_auto_add_executable(${example_name}
      ${example_file}
    )
    ament_target_dependencies(${example_name}
      ${${PROJECT_NAME}_FOUND_TEST_DEPENDS}
    )
  endforeach()

endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  test_data
)
