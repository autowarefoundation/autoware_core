cmake_minimum_required(VERSION 3.14)
project(autoware_gnss_poser)

find_package(autoware_cmake REQUIRED)
autoware_package()

## Find non-ROS library
find_package(GeographicLib REQUIRED)

# Logic layer: rclcpp-free helpers of the pose computation.
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gnss_poser.cpp
)

# ROS layer: parameters, pub/sub, TF lookup and broadcast.
ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/gnss_poser_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::gnss_poser::GnssPoserNode"
  EXECUTABLE gnss_poser
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  # The helpers only do message arithmetic, so this test needs no ROS context and can use a plain
  # gtest instead of a ROS-isolated one.
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_gnss_poser.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)

  # ROS-based tests can be executed only when agnocast is disabled.
  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    # Integration test of the node over real topics; the pose computation is covered above.
    ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_node
      test/test_gnss_poser_node.cpp
      TIMEOUT 120
    )
    target_link_libraries(test_${PROJECT_NAME}_node ${PROJECT_NAME}_ros)
    target_include_directories(test_${PROJECT_NAME}_node PRIVATE src)
    # Lets the test load the shipped parameter file from the source tree.
    target_compile_definitions(test_${PROJECT_NAME}_node
      PRIVATE GNSS_POSER_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config"
    )
  endif()
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
