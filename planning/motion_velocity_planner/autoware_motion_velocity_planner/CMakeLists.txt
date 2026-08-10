cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_planner)

find_package(autoware_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::motion_velocity_planner::MotionVelocityPlannerNode"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR SingleThreadedExecutor
  AGNOCAST_EXECUTOR CallbackIsolatedAgnocastExecutor
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_node
    test/test_node.cpp
  )
  target_link_libraries(test_node
    ${PROJECT_NAME}_lib
    yaml-cpp
  )

  ament_auto_add_gtest(test_node_utils
    test/test_node_utils.cpp
  )
  target_link_libraries(test_node_utils
    ${PROJECT_NAME}_lib
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  test_config
  test_data
  test_map
)
