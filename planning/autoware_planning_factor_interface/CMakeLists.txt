cmake_minimum_required(VERSION 3.14)
project(autoware_planning_factor_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_planning_factor_interface SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_planning_factor_interface.cpp
  )
endif()

autoware_ament_auto_package()
