cmake_minimum_required(VERSION 3.14)
project(autoware_downsample_filters)

find_package(autoware_cmake REQUIRED)

autoware_package()

set(COMMON base_and_utilities)
set(PICKUP_BASED_VOXEL_GRID pickup_based_voxel_grid_downsample_filter)
set(RANDOM random_downsample_filter)
set(VOXEL_GRID voxel_grid_downsample_filter)

add_library(${COMMON} SHARED
  src/filter.cpp
  src/memory.cpp
)

target_include_directories(${COMMON} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  ${autoware_point_types_INCLUDE_DIRS}
)

ament_target_dependencies(${COMMON}
  pcl_conversions
  rclcpp
  autoware_utils_debug
  autoware_utils_system
  autoware_point_types
  managed_transform_buffer
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pickup_based_voxel_grid_downsample_filter_node.cpp
  src/random_downsample_filter_node.cpp
  src/faster_voxel_grid_downsample_filter.cpp
  src/voxel_grid_downsample_filter_node.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${COMMON}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::PickupBasedVoxelGridDownsampleFilter"
  EXECUTABLE ${PICKUP_BASED_VOXEL_GRID}_node)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::RandomDownsampleFilter"
  EXECUTABLE ${RANDOM}_node)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::VoxelGridDownsampleFilter"
  EXECUTABLE ${VOXEL_GRID}_node)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)