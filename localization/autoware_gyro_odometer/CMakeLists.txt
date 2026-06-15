cmake_minimum_required(VERSION 3.14)
project(autoware_gyro_odometer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gyro_odometer_core.cpp
  src/gyro_odometer_fusion.cpp
)

target_link_libraries(${PROJECT_NAME} fmt)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)

if(BUILD_TESTING)
  # The pub/sub integration test spins GyroOdometerNode with rclcpp::spin_some, which neither
  # accepts an autoware::agnocast_wrapper::Node nor drives Agnocast callbacks. Build it only in
  # non-Agnocast builds; the helper and fusion tests (pure functions) always build.
  if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_add_ros_isolated_gtest(test_gyro_odometer
      test/test_main.cpp
      test/test_gyro_odometer_helper.cpp
      test/test_gyro_odometer_fusion.cpp
    )
  else()
    ament_add_ros_isolated_gtest(test_gyro_odometer
      test/test_main.cpp
      test/test_gyro_odometer_pubsub.cpp
      test/test_gyro_odometer_helper.cpp
      test/test_gyro_odometer_fusion.cpp
    )
  endif()
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
