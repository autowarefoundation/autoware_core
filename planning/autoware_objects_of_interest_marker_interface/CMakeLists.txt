cmake_minimum_required(VERSION 3.5)
project(autoware_objects_of_interest_marker_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(autoware_objects_of_interest_marker_interface SHARED
  src/coloring.cpp
  src/objects_of_interest_marker_interface.cpp
  src/marker_utils.cpp
)

# Test
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_objects_of_interest_marker_interface
    test/test_objects_of_interest_marker_interface.cpp
  )
  target_link_libraries(test_objects_of_interest_marker_interface
    autoware_objects_of_interest_marker_interface
  )
  ament_target_dependencies(test_objects_of_interest_marker_interface
    rclcpp
    geometry_msgs
    std_msgs
    visualization_msgs
    autoware_perception_msgs
  )
endif()

autoware_ament_auto_package()
