cmake_minimum_required(VERSION 3.14)
project(autoware_core_localization)

find_package(autoware_cmake REQUIRED)
autoware_package()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
