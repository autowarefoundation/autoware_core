cmake_minimum_required(VERSION 3.14)
project(autoware_downsample_filters)

find_package(autoware_cmake REQUIRED)

autoware_package()

set(RANDOM random_downsample_filter)
set(VOXEL_GRID voxel_grid_downsample_filter)


ament_auto_add_library(${PROJECT_NAME} SHARED
  src/random_downsample_filter/random_downsample_filter_node.cpp
  src/voxel_grid_downsample_filter/voxel_grid_downsample_filter_node.cpp
  src/voxel_grid_downsample_filter/faster_voxel_grid_downsample_filter.cpp
  src/voxel_grid_downsample_filter/memory.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::RandomDownsampleFilter"
  EXECUTABLE ${RANDOM}_node)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::downsample_filters::VoxelGridDownsampleFilter"
  EXECUTABLE ${VOXEL_GRID}_node)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
