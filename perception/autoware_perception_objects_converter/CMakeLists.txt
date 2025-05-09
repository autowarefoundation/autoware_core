cmake_minimum_required(VERSION 3.14)
project(autoware_perception_objects_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

## Build
ament_auto_add_library(detected_to_predicted_objects_converter SHARED
  src/detected_to_predicted_objects_converter.cpp
)

rclcpp_components_register_node(detected_to_predicted_objects_converter
  PLUGIN "autoware::perception_objects_converter::DetectedToPredictedObjectsConverter"
  EXECUTABLE detected_to_predicted_objects_converter_node
)

## Install
ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
