cmake_minimum_required(VERSION 3.14)
project(autoware_twist2accel)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/accel_estimator.cpp
  src/twist2accel.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::twist2accel::Twist2Accel"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_accel_estimator
    test/test_accel_estimator.cpp
  )
  target_link_libraries(test_accel_estimator ${PROJECT_NAME})
  target_include_directories(test_accel_estimator PRIVATE src)

  ament_add_ros_isolated_gtest(test_twist2accel_node
    test/test_twist2accel_node.cpp
  )
  target_link_libraries(test_twist2accel_node ${PROJECT_NAME})
  target_include_directories(test_twist2accel_node PRIVATE src)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
