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
  ament_add_ros_isolated_gtest(test_gyro_odometer_fusion
    test/test_main.cpp
    test/test_gyro_odometer_fusion.cpp
  )
  ament_target_dependencies(test_gyro_odometer_fusion
    rclcpp
  )
  target_link_libraries(test_gyro_odometer_fusion
    ${PROJECT_NAME}
  )
  target_include_directories(test_gyro_odometer_fusion PRIVATE
    src
  )

  if(NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_add_ros_isolated_gtest(test_gyro_odometer_pubsub
      test/test_main.cpp
      test/test_gyro_odometer_pubsub.cpp
      test/test_gyro_odometer_helper.cpp
    )
    ament_target_dependencies(test_gyro_odometer_pubsub
      rclcpp
    )
    target_link_libraries(test_gyro_odometer_pubsub
      ${PROJECT_NAME}
    )
    target_include_directories(test_gyro_odometer_pubsub PRIVATE
      src
    )
  endif()
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
