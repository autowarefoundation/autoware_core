cmake_minimum_required(VERSION 3.14)
project(autoware_vehicle_velocity_converter)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/vehicle_velocity_converter.cpp
)

ament_auto_add_library(${PROJECT_NAME}_ros SHARED
  src/vehicle_velocity_converter_node.cpp
)

target_link_libraries(${PROJECT_NAME}_ros
  ${PROJECT_NAME}
)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}_ros
  PLUGIN "autoware::vehicle_velocity_converter::VehicleVelocityConverterNode"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_vehicle_velocity_converter.cpp
  )
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_include_directories(test_${PROJECT_NAME} PRIVATE src)

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}_node
    test/test_vehicle_velocity_converter_node.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}_node ${PROJECT_NAME} ${PROJECT_NAME}_ros)
  target_include_directories(test_${PROJECT_NAME}_node PRIVATE src)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
