cmake_minimum_required(VERSION 3.14)
project(autoware_planning_factor_interface)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(autoware_planning_factor_interface SHARED
  DIRECTORY src
)

if(BUILD_TESTING)
  find_package(autoware_agnocast_wrapper REQUIRED)
  ament_auto_add_gtest(test_${PROJECT_NAME}
    test/test_planning_factor_interface.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${autoware_agnocast_wrapper_LIBRARIES}
  )
  target_include_directories(test_${PROJECT_NAME} PRIVATE
    ${autoware_agnocast_wrapper_INCLUDE_DIRS}
  )
endif()

autoware_ament_auto_package()
