cmake_minimum_required(VERSION 3.14)
project(autoware_stop_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
)

ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/stop_filter_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::stop_filter::StopFilterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_stop_filter_node
    test/test_stop_filter_node.cpp
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
