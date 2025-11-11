cmake_minimum_required(VERSION 3.14)
project(autoware_crop_box_filter)

find_package(autoware_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)

autoware_package()

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(crop_box_filter_node SHARED
  src/crop_box_filter_node.cpp
)

rclcpp_components_register_node(crop_box_filter_node
  PLUGIN "autoware::crop_box_filter::CropBoxFilter"
  EXECUTABLE crop_box_filter)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_crop_box_filter_node
    test/test_crop_box_filter_node.cpp
  )

  ament_auto_add_gtest(test_crop_box_filter_integration
    test/test_crop_box_filter_integration.cpp
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  test
)
