cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)

if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(agnocastlib REQUIRED)
endif()

include_directories(
  include
)

ament_export_include_directories(
  include
)

if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_export_dependencies(
    agnocastlib
  )
endif()

list(APPEND ${PROJECT_NAME}_CONFIG_EXTRAS
  "cmake/agnocast_wrapper_config_extras.cmake"
)

install(
  FILES cmake/agnocast_wrapper_config_extras.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

autoware_ament_auto_package()
