cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(autoware_utils_rclcpp REQUIRED)

if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(agnocastlib REQUIRED)
endif()

include_directories(
  include
)

if("$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_auto_add_library(${PROJECT_NAME} SHARED
    src/node.cpp
  )
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    agnocastlib
    autoware_utils_rclcpp
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_AGNOCAST_ENABLED)
endif()

ament_export_include_directories(
  include
)

ament_export_dependencies(
  autoware_utils
  rclcpp
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
