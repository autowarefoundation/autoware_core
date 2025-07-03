cmake_minimum_required(VERSION 3.14)

project(autoware_trajectory)


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
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(autoware_motion_utils REQUIRED)
  include_directories(${autoware_motion_utils_INCLUDE_DIRS})

  set(test_files
    test/test_shift.cpp
    test/test_helpers.cpp
    test/test_interpolator.cpp
    test/test_pretty_build.cpp
    test/test_reference_path.cpp
    test/test_trajectory_container.cpp
    test/test_trajectory_container_trajectory_point.cpp
    test/test_utils_find_nearest.cpp
  )
  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_autoware_trajectory ${test_files})

  target_link_libraries(test_autoware_trajectory
    autoware_trajectory
    ${autoware_motion_utils_LIBRARIES}
  )

  # Examples
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
  )

  add_definitions("-Wno-attributes")
  find_package(ament_index_cpp REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  find_package(python_cmake_module REQUIRED)
  find_package(
    Python3
    COMPONENTS Interpreter Development
    REQUIRED)
  find_package(autoware_pyplot REQUIRED)
  find_package(autoware_utils_geometry REQUIRED)
  find_package(range-v3 REQUIRED)
  find_package(autoware_test_utils REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)
  include_directories(${autoware_index_cpp_INCLUDE_DIRS})
  include_directories(${autoware_pyplot_INCLUDE_DIRS})
  include_directories(${autoware_test_utils_INCLUDE_DIRS})
  include_directories(${autoware_utils_geometry_INCLUDE_DIRS})
  foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    ament_auto_add_executable(${example_name}
      ${example_file}
    )
    target_link_libraries(${example_name}
      autoware_trajectory
      ${autoware_index_cpp_LIBRARIES}
      ${autoware_utils_geometry_LIBRARIES}
      ${autoware_motion_utils_LIBRARIES}
      ${autoware_pyplot_LIBRARIES}
      range-v3::range-v3
      Python3::Python
      pybind11::embed
      ${autoware_test_utils_LIBRARIES}
    )
  endforeach()
endif()

ament_auto_package()
