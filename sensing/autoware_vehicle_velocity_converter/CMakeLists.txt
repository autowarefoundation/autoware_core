cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_velocity_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ROS-independent library
ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  lib/vehicle_velocity_converter.cpp
)

# ROS node
ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/vehicle_velocity_converter_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}_lib
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::vehicle_velocity_converter::VehicleVelocityConverterNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # ROS-independent unit tests
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}_lib
    test/test_vehicle_velocity_converter.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_lib
    ${PROJECT_NAME}_lib
  )
endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
