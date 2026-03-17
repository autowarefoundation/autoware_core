cmake_minimum_required(VERSION 3.14)
project(autoware_agnocast_wrapper)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(rclcpp REQUIRED)
find_package(autoware_utils_rclcpp REQUIRED)
find_package(message_filters REQUIRED)

if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  find_package(agnocastlib REQUIRED)
endif()

include_directories(
  include
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
if("$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    agnocastlib
    autoware_utils_rclcpp
    message_filters
  )
  target_compile_definitions(${PROJECT_NAME} PUBLIC USE_AGNOCAST_ENABLED)
else()
  ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    autoware_utils_rclcpp
    message_filters
  )
endif()

ament_export_include_directories(
  include
)

ament_export_dependencies(
  autoware_utils_rclcpp
  class_loader
  message_filters
  rclcpp
  rclcpp_components
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

install(
  FILES cmake/autoware_agnocast_wrapper_register_node.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

install(
  DIRECTORY templates
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

autoware_ament_auto_package()
