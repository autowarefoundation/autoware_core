cmake_minimum_required(VERSION 3.14)
project(autoware_gyro_odometer)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/gyro_odometer_diagnostics.cpp
  src/gyro_odometer_fusion.cpp
)

target_link_libraries(${PROJECT_NAME} fmt)

# ROS layer: parameters, pub/sub, TF lookup and the diagnostics timer.
ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/gyro_odometer_core.cpp
)

target_link_libraries(${PROJECT_NAME}_ros ${PROJECT_NAME})

if(BUILD_TESTING)
  # The fusion functions only do message arithmetic, so this test needs no ROS context and can use
  # a plain gtest instead of a ROS-isolated one.
  ament_add_gtest(test_gyro_odometer_fusion
    test/test_gyro_odometer_fusion.cpp
  )
  ament_target_dependencies(test_gyro_odometer_fusion
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
  )
  target_link_libraries(test_gyro_odometer_fusion
    ${PROJECT_NAME}
  )
  target_include_directories(test_gyro_odometer_fusion PRIVATE
    src
  )

# The diagnostics decision is a pure function too, so the same plain-gtest treatment applies.
  ament_add_gtest(test_gyro_odometer_diagnostics
    test/test_gyro_odometer_diagnostics.cpp
  )
  ament_target_dependencies(test_gyro_odometer_diagnostics
    ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS}
  )
  target_link_libraries(test_gyro_odometer_diagnostics
    ${PROJECT_NAME}
  )
  target_include_directories(test_gyro_odometer_diagnostics PRIVATE
    src
  )

  if(NOT "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    ament_add_ros_isolated_gtest(test_gyro_odometer
      test/test_gyro_odometer_pubsub.cpp
    )
    ament_target_dependencies(test_gyro_odometer
      rclcpp
    )
    target_link_libraries(test_gyro_odometer
      ${PROJECT_NAME}
      ${PROJECT_NAME}_ros //ROS-based test can be executed only when agnocast is disabled
    )
    target_include_directories(test_gyro_odometer PRIVATE
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
