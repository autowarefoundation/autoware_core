cmake_minimum_required(VERSION 3.14)
project(autoware_gyro_odometer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gyro_odometer_core.cpp
  src/gyro_odometer_fusion.cpp
)

target_link_libraries(${PROJECT_NAME} fmt)

if(BUILD_TESTING)
  set(test_sources
    test/test_main.cpp
    test/test_gyro_odometer_helper.cpp
    test/test_gyro_odometer_fusion.cpp
  )
  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    list(APPEND test_sources test/test_gyro_odometer_pubsub.cpp)
  endif()

  ament_add_ros_isolated_gtest(test_gyro_odometer ${test_sources})
  ament_target_dependencies(test_gyro_odometer
    rclcpp
  )
  target_link_libraries(test_gyro_odometer
    ${PROJECT_NAME}
  )
  target_include_directories(test_gyro_odometer PRIVATE
    src
  )
  ament_target_dependencies(test_gyro_odometer autoware_agnocast_wrapper)
  autoware_agnocast_wrapper_setup(test_gyro_odometer)
endif()

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::gyro_odometer::GyroOdometerNode"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
