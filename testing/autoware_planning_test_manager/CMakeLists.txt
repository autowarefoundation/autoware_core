cmake_minimum_required(VERSION 3.14)
project(autoware_planning_test_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_planning_test_manager SHARED
  src/autoware_planning_test_manager.cpp
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_autoware_planning_test_manager
    test/test_planning_test_manager.cpp
  )
endif()

autoware_ament_auto_package()
