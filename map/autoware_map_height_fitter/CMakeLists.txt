cmake_minimum_required(VERSION 3.14)
project(autoware_map_height_fitter)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(PCL REQUIRED COMPONENTS common)
find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/map_height_fitter_kernel.cpp
  src/map_height_fitter.cpp
  src/map_height_fitter_node.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

# MapHeightFitterNode derives from autoware::agnocast_wrapper::Node, which requires an AgnocastOnly
# executor, so it runs as a standalone executable rather than inside a component container.
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::map_height_fitter::MapHeightFitterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyMultiThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}_kernel
    test/test_map_height_fitter_kernel.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_kernel ${PROJECT_NAME} ${PCL_LIBRARIES})
  # map_height_fitter_kernel.hpp is an internal (non-installed) header under src/.
  target_include_directories(test_${PROJECT_NAME}_kernel PRIVATE src)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_fit
    test/test_map_height_fitter_fit.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_fit ${PROJECT_NAME} ${PCL_LIBRARIES})
  ament_target_dependencies(test_${PROJECT_NAME}_fit autoware_agnocast_wrapper)
  autoware_agnocast_wrapper_setup(test_${PROJECT_NAME}_fit)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
