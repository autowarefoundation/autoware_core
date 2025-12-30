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

ament_auto_add_library(autoware_trajectory SHARED
  src/threshold.cpp
  src/point.cpp
  src/pose.cpp
  src/path_point.cpp
  src/path_point_with_lane_id.cpp
  src/trajectory_point.cpp
  src/detail/logging.cpp
  src/detail/types.cpp
  src/detail/util.cpp
  src/interpolator/akima_spline.cpp
  src/interpolator/cubic_spline.cpp
  src/interpolator/lane_ids_interpolator.cpp
  src/interpolator/linear.cpp
  src/interpolator/result.cpp
  src/interpolator/spherical_linear.cpp
  src/utils/closest.cpp
  src/utils/crossed.cpp
  src/utils/find_intervals.cpp
  src/utils/pretty_build.cpp
  src/utils/shift.cpp
  src/utils/reference_path.cpp
  src/utils/find_nearest.cpp
  src/utils/footprint.cpp
  src/utils/velocity.cpp
)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()

  set(test_files
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
  )
  foreach(test_file ${test_files})
    get_filename_component(test_name ${test_file} NAME_WE)
    ament_auto_add_gtest(${test_name}
      ${test_file}
    )
    target_link_libraries(${test_name}
      fmt::fmt # need this somehow
      Python3::Python
      pybind11::embed
      yaml-cpp
    )
  endforeach()
endif()

# Examples
find_package(
  Python3
  COMPONENTS Interpreter Development
  REQUIRED)
find_package(pybind11_vendor REQUIRED)
find_package(pybind11 REQUIRED)
find_package(autoware_pyplot REQUIRED)
find_package(autoware_motion_utils REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(autoware_test_utils REQUIRED)
find_package(autoware_utils_geometry REQUIRED)
find_package(range-v3 REQUIRED)

include_directories(${autoware_pyplot_INCLUDE_DIRS})
include_directories(${autoware_motion_utils_INCLUDE_DIRS})
include_directories(${autoware_index_cpp_INCLUDE_DIRS})
include_directories(${autoware_test_utils_INCLUDE_DIRS})
include_directories(${autoware_utils_geometry_INCLUDE_DIRS})

set(example_files
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
)

foreach(example_file ${example_files})
  get_filename_component(example_name ${example_file} NAME_WE)
  ament_auto_add_executable(${example_name}
    ${example_file}
  )
  target_link_libraries(${example_name}
    autoware_trajectory
    ${autoware_pyplot_LIBRARIES}
    ${autoware_motion_utils_LIBRARIES}
    ${autoware_index_cpp_LIBRARIES}
    ${autoware_test_utils_LIBRARIES}
    ${autoware_utils_geometry_LIBRARIES}
    range-v3::range-v3
    Python3::Python
    pybind11::embed
    yaml-cpp
  )
endforeach()

autoware_ament_auto_package(INSTALL_TO_SHARE
  test_data
)
