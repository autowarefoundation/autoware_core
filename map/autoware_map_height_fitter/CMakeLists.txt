cmake_minimum_required(VERSION 3.14)
project(autoware_map_height_fitter)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_height_fitter.cpp
  src/map_height_fitter_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::map_height_fitter::MapHeightFitterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
