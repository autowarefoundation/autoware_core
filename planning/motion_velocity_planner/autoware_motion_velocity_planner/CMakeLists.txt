cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_planner)

find_package(autoware_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::motion_velocity_planner::MotionVelocityPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
