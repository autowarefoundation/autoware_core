cmake_minimum_required(VERSION 3.8)
project(autoware_test_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/test_node.cpp)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::test_node::TestNode"
  EXECUTABLE ${PROJECT_NAME}_node)

autoware_ament_auto_package(INSTALL_TO_SHARE
  launch)
