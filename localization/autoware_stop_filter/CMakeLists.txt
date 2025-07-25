cmake_minimum_required(VERSION 3.14)
project(autoware_stop_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/stop_filter.cpp
)

ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/stop_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

rclcpp_components_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::stop_filter::StopFilterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_stop_filter_node
    test/test_stop_filter.cpp
    test/test_stop_filter_node.cpp
  )
  target_link_libraries(test_stop_filter_node ${PROJECT_NAME} ${PROJECT_NAME}_ros)
  target_include_directories(test_stop_filter_node PRIVATE src)
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
