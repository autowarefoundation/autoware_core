cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_obstacle_stop_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

# Library configuration
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# Test configuration
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)

# Add test executable
ament_add_gtest(test_obstacle_stop_module_outside_check
  test/test_obstacle_stop_module_outside_check.cpp
  test/test_utils.cpp
)

# Set test dependencies
target_link_libraries(test_obstacle_stop_module_outside_check
  ${PROJECT_NAME}
)

# Set test include directories
target_include_directories(test_obstacle_stop_module_outside_check
  PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR}/test
)

ament_auto_package(INSTALL_TO_SHARE config)
