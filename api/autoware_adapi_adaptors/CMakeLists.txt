cmake_minimum_required(VERSION 3.14)
project(autoware_adapi_adaptors)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/initial_pose_adaptor.cpp
  src/routing_adaptor.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

# InitialPoseAdaptor derives from autoware::agnocast_wrapper::Node, which requires an AgnocastOnly
# executor, so it runs as a standalone executable rather than inside a component container.
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::adapi_adaptors::InitialPoseAdaptor"
  EXECUTABLE initial_pose_adaptor_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyMultiThreadedExecutor
)

# RoutingAdaptor derives from autoware::agnocast_wrapper::Node, which requires an AgnocastOnly
# executor, so it runs as a standalone executable rather than inside a component container.
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::adapi_adaptors::RoutingAdaptor"
  EXECUTABLE routing_adaptor_node
  AGNOCAST_EXECUTOR AgnocastOnlySingleThreadedExecutor
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_helpers.cpp
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE config launch)
