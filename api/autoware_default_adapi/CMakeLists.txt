cmake_minimum_required(VERSION 3.14)
project(autoware_default_adapi)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/interface.cpp
  src/localization.cpp
  src/routing.cpp
  src/utils/localization_conversion.cpp
  src/utils/route_conversion.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

# These nodes derive from autoware::agnocast_wrapper::Node, which requires an AgnocastOnly
# executor, so they run as standalone executables rather than inside a component container.
# They were composed into component_container_mt, so keep a multi-threaded executor:
# LocalizationNode blocks on a client call from inside a service callback and would deadlock
# on a single-threaded one.
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::InterfaceNode"
  EXECUTABLE interface_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyMultiThreadedExecutor
)
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::LocalizationNode"
  EXECUTABLE localization_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyMultiThreadedExecutor
)
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::default_adapi::RoutingNode"
  EXECUTABLE routing_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyMultiThreadedExecutor
)

if(BUILD_TESTING)
  add_launch_test(test/main.test.py)

  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_conversion.cpp
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE config launch test)
