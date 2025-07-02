cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/localization.cpp
  src/routing.cpp
  src/utils/localization_conversion.cpp
  src/utils/route_conversion.cpp
)

rclcpp_components_register_nodes(${PROJECT_NAME}
  "autoware::default_adapi::InterfaceNode"
  "autoware::default_adapi::LocalizationNode"
  "autoware::default_adapi::RoutingNode"
)

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)
endif()

ament_auto_package(INSTALL_TO_SHARE config launch test)
