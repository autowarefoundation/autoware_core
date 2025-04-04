cmake_minimum_required(VERSION 3.14)

project(autoware_trajectory)


find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_trajectory SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(autoware_motion_utils REQUIRED)
  include_directories(${autoware_motion_utils_INCLUDE_DIRS})

  file(GLOB_RECURSE test_files test/*.cpp)

  ament_add_ros_isolated_gtest(test_autoware_trajectory ${test_files})

  target_link_libraries(test_autoware_trajectory
    autoware_trajectory
    ${autoware_motion_utils_LIBRARIES}
  )

  # Examples
  file(GLOB_RECURSE example_files examples/*.cpp)

  add_definitions("-Wno-attributes")
  find_package(ament_lint_auto REQUIRED)
  find_package(autoware_pyplot REQUIRED)
  find_package(autoware_utils_geometry REQUIRED)
  find_package(range-v3 REQUIRED)
  find_package(autoware_test_utils REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(pybind11 REQUIRED)
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
      ${autoware_utils_geometry_LIBRARIES}
      ${autoware_motion_utils_LIBRARIES}
      ${autoware_pyplot_LIBRARIES}
      range-v3::range-v3
      pybind11::embed
      ${autoware_test_utils_LIBRARIES}
    )
  endforeach()
endif()

ament_auto_package()
