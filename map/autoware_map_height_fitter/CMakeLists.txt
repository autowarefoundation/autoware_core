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

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::map_height_fitter::MapHeightFitterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}_kernel
    test/test_map_height_fitter_kernel.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_kernel ${PROJECT_NAME} ${PCL_LIBRARIES})
  # map_height_fitter_kernel.hpp is an internal (non-installed) header under src/.
  target_include_directories(test_${PROJECT_NAME}_kernel PRIVATE src)

  # Agnocast deliberately aborts the process when a Publisher or a Subscription is created
  # without libagnocast_heaphook.so in LD_PRELOAD (validate_ld_preload() in agnocast_utils.cpp).
  # We do not want the tests to depend on the heaphook or on the kernel module, so the tests
  # that create publishers/subscriptions are the ones skipped at ENABLE_AGNOCAST=1.
  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_fit
      test/test_map_height_fitter_fit.cpp
    )
    target_link_libraries(test_${PROJECT_NAME}_fit ${PROJECT_NAME} ${PCL_LIBRARIES})
    ament_target_dependencies(test_${PROJECT_NAME}_fit autoware_agnocast_wrapper)
    autoware_agnocast_wrapper_setup(test_${PROJECT_NAME}_fit)
  endif()
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
