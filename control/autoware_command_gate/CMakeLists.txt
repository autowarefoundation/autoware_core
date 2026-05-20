cmake_minimum_required(VERSION 3.14)
project(autoware_command_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/autoware_command_gate.cpp
  src/command_gate_mode_builder.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    src
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::control::command_gate::AutowareCommandGateNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_ros REQUIRED)
  find_package(class_loader REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)

  ament_add_gtest(test_command_gate_mode_builder
    test/unit/test_command_gate_mode_builder.cpp
  )
  target_link_libraries(test_command_gate_mode_builder
    ${PROJECT_NAME}_node
  )
  ament_target_dependencies(test_command_gate_mode_builder
    autoware_adapi_v1_msgs
    autoware_vehicle_msgs
    builtin_interfaces
  )

  ament_add_ros_isolated_gtest(test_ros_integration
    test/integration/test_ros_integration.cpp
  )
  target_link_libraries(test_ros_integration
    ${PROJECT_NAME}_node
  )
  ament_target_dependencies(test_ros_integration
    autoware_adapi_v1_msgs
    autoware_system_msgs
    autoware_vehicle_msgs
    class_loader
    rclcpp
    rclcpp_components
  )
endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
