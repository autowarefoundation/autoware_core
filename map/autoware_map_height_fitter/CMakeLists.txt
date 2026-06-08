cmake_minimum_required(VERSION 3.14)
project(autoware_map_height_fitter)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(PCL REQUIRED COMPONENTS common)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_height_fitter_kernel.cpp
  src/map_height_fitter.cpp
  src/map_height_fitter_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::map_height_fitter::MapHeightFitterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}_kernel
    test/test_map_height_fitter_kernel.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_kernel ${PROJECT_NAME} ${PCL_LIBRARIES})
  # map_height_fitter_kernel.hpp is an internal (non-installed) header under src/.
  target_include_directories(test_${PROJECT_NAME}_kernel PRIVATE src)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
