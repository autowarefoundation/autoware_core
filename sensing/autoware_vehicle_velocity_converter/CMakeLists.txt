cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_velocity_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vehicle_velocity_converter.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::vehicle_velocity_converter::VehicleVelocityConverter"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_${PROJECT_NAME}
    test/test_vehicle_velocity_converter.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
