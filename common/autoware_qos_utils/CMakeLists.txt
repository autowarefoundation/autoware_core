cmake_minimum_required(VERSION 3.8)
project(autoware_qos_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

autoware_ament_auto_package(INSTALL_TO_SHARE)
