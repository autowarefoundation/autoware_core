cmake_minimum_required(VERSION 3.14)
project(autoware_gnss_poser)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(GeographicLib REQUIRED)

set(GNSS_POSER_HEADERS
  include/autoware/gnss_poser/gnss_poser_node.hpp
)

ament_auto_add_library(gnss_poser_node SHARED
  src/gnss_poser_node.cpp
  ${GNSS_POSER_HEADERS}
)

rclcpp_components_register_node(gnss_poser_node
  PLUGIN "autoware::gnss_poser::GNSSPoser"
  EXECUTABLE gnss_poser
)

if(BUILD_TESTING)
  set(TEST_SOURCES
    test/test_gnss_poser_node.cpp
  )
  set(TEST_GNSS_POSER_EXE test_gnss_poser)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${TEST_SOURCES})
  target_link_libraries(test_${PROJECT_NAME} gnss_poser_node)
endif()

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
