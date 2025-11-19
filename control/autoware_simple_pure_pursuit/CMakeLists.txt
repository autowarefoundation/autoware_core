cmake_minimum_required(VERSION 3.8)
project(autoware_simple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::control::simple_pure_pursuit::SimplePurePursuitNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(${PROJECT_NAME}_test
    test/test_simple_pure_pursuit.cpp
  )
  target_link_libraries(${PROJECT_NAME}_test
    ${PROJECT_NAME}_lib
  )
endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
    launch
    config
)
