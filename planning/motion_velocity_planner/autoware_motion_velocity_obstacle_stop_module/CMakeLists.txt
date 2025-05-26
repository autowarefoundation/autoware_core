cmake_minimum_required(VERSION 3.5)
project(autoware_motion_velocity_obstacle_stop_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(autoware_cmake REQUIRED)
find_package(autoware_auto_planning_msgs REQUIRED)
find_package(autoware_motion_velocity_planner_common REQUIRED)
find_package(autoware_utils REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build the library
add_library(${PROJECT_NAME} SHARED
  src/obstacle_stop_module.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  autoware_auto_planning_msgs
  autoware_motion_velocity_planner_common
  autoware_utils
  geometry_msgs
  rclcpp
  tf2_ros
)

# Build the tests
add_executable(test_obstacle_stop_module
  test/test_obstacle_stop_module.cpp
)

target_link_libraries(test_obstacle_stop_module
  ${PROJECT_NAME}
  gtest
  gtest_main
)

ament_target_dependencies(test_obstacle_stop_module
  autoware_auto_planning_msgs
  autoware_motion_velocity_planner_common
  autoware_utils
  geometry_msgs
  rclcpp
  tf2_ros
)

ament_add_gtest(test_obstacle_stop_module)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
