cmake_minimum_required(VERSION 3.14)
project(autoware_localization_fusion_host)

find_package(autoware_cmake REQUIRED)
autoware_package()

pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

ament_auto_add_library(${PROJECT_NAME}_plugins SHARED
  src/stop_filter_plugin.cpp
  src/twist2accel_plugin.cpp
)

ament_auto_add_library(${PROJECT_NAME}_component SHARED
  src/localization_fusion_host.cpp
  src/ekf_fusion_core.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "autoware::localization_fusion_host::LocalizationFusionHost"
  EXECUTABLE ${PROJECT_NAME}
  EXECUTOR SingleThreadedExecutor
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
