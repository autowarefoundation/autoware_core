cmake_minimum_required(VERSION 3.14)
project(autoware_ground_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
  src/ground_filter.cpp
  src/sanity_check.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ========== Ground Filter ==========
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ground_filter::GroundFilterComponent"
  EXECUTABLE ground_filter_node)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
