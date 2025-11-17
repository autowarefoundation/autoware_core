cmake_minimum_required(VERSION 3.8)
project(autoware_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED src/node.cpp)

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_FILES test/*.cpp)

  foreach(TEST_FILE ${TEST_FILES})
    # Get the test name without directory and extension
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Add each test separately
    ament_add_ros_isolated_gtest(${TEST_NAME} ${TEST_FILE} TIMEOUT 10)
    target_include_directories(${TEST_NAME} PRIVATE src/include)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
    ament_target_dependencies(${TEST_NAME}
      rclcpp)
  endforeach()
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE)
