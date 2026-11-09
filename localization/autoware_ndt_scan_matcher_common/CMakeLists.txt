cmake_minimum_required(VERSION 3.14)
project(autoware_ndt_scan_matcher_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

# This package has headers only.

# The public headers include Eigen.
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
ament_export_dependencies(eigen3_cmake_module Eigen3)

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
