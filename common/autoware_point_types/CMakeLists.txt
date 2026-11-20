cmake_minimum_required(VERSION 3.14)
project(autoware_point_types)

find_package(autoware_cmake REQUIRED)
autoware_package()

include_directories(
  include
  SYSTEM
  ${PCL_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_autoware_point_types
    test/test_point_types.cpp
  )
  target_include_directories(test_autoware_point_types
    PRIVATE include
  )
  target_link_libraries(test_autoware_point_types
    ${point_cloud_msg_wrapper_LIBRARIES}
  )
  target_include_directories(test_autoware_point_types PRIVATE
    ${point_cloud_msg_wrapper_INCLUDE_DIRS}
  )

  ament_add_ros_isolated_gtest(test_autoware_point_types_memory
    test/test_memory.cpp
  )
  target_include_directories(test_autoware_point_types_memory
    PRIVATE include
  )
  target_link_libraries(test_autoware_point_types_memory
    ${point_cloud_msg_wrapper_LIBRARIES}
    ${sensor_msgs_TARGETS}
  )
  target_include_directories(test_autoware_point_types_memory PRIVATE
    ${point_cloud_msg_wrapper_INCLUDE_DIRS}
  )
endif()

autoware_ament_auto_package()
