cmake_minimum_required(VERSION 3.14)
project(autoware_motion_velocity_obstacle_stop_module)

find_package(autoware_cmake REQUIRED)
autoware_package()
pluginlib_export_plugin_description_file(autoware_motion_velocity_planner plugins.xml)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY
  src
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    # test/test_obstacle_stop_module_outside_check.cpp
    test/test_planning_factor.cpp
    # test/_test_utils.cpp
  )

  target_link_libraries(
    test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )

endif()

ament_auto_package(INSTALL_TO_SHARE config test/test_config)
