cmake_minimum_required(VERSION 3.14)
project(autoware_global_parameter_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_global_params_launch test/test_global_params_launch.py)
endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
