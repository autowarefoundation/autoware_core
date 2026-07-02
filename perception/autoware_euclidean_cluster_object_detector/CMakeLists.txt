cmake_minimum_required(VERSION 3.14)
project(autoware_euclidean_cluster_object_detector)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(PCL REQUIRED)

include_directories(
  src
  lib
  SYSTEM
  ${PCL_COMMON_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  src/euclidean_cluster_object_detector.cpp
  lib/ros_conversions.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${PCL_LIBRARIES}
)

target_include_directories(${PROJECT_NAME}_lib
  PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/lib>
)

ament_auto_add_library(${PROJECT_NAME}_node_core SHARED
  src/euclidean_cluster_object_detector_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node_core
  ${PCL_LIBRARIES}
  ${PROJECT_NAME}_lib
)

rclcpp_components_register_node(${PROJECT_NAME}_node_core
  PLUGIN "autoware::euclidean_cluster::EuclideanClusterObjectDetectorNode"
  EXECUTABLE euclidean_cluster_object_detector_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_auto_add_gtest(test_euclidean_cluster_object_detector_integration
    test/test_euclidean_cluster_object_detector_integration.cpp
  )
  target_link_libraries(test_euclidean_cluster_object_detector_integration
    ${PROJECT_NAME}_node_core
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
    launch
    config
    test
)
