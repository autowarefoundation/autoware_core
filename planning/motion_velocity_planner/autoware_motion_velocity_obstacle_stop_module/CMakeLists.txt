cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_obstacle_stop_module)

find_package(autoware_cmake REQUIRED)

autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

# Library configuration
ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  # Add test executable
  ament_add_ros_isolated_gtest(test_obstacle_stop_module_outside_check
    test/test_utils.cpp
    test/test_obstacle_stop_module_outside_check.cpp
  )

  ament_add_ros_isolated_gtest(test_types
    test/test_types.cpp
  )

  ament_add_ros_isolated_gtest(test_parameters
    test/test_parameters.cpp
  )

  ament_add_ros_isolated_gtest(test_planning_factor
    test/test_planning_factor.cpp
  )

  target_link_libraries(test_obstacle_stop_module_outside_check ${PROJECT_NAME})
  target_link_libraries(test_types ${PROJECT_NAME})
  target_link_libraries(test_parameters ${PROJECT_NAME})
  target_link_libraries(test_planning_factor ${PROJECT_NAME})

endif()

ament_auto_package(INSTALL_TO_SHARE config)
