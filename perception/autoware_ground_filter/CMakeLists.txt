cmake_minimum_required(VERSION 3.14)
project(autoware_ground_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ground_filter_node.cpp
  src/ground_filter.cpp
  src/sanity_check.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ========== Ground Filter ==========
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ground_filter::GroundFilterComponent"
  EXECUTABLE ground_filter_node)

if(BUILD_TESTING)
  find_package(autoware_test_utils REQUIRED)

  ament_add_gtest(test_ground_filter
    test/test_ground_filter.cpp
  )
  target_include_directories(test_ground_filter PRIVATE src)
  target_link_libraries(test_ground_filter
    ${PROJECT_NAME}
  )

  ament_add_gtest(test_sanity_check
    test/test_sanity_check.cpp
  )
  target_include_directories(test_sanity_check PRIVATE src)
  target_link_libraries(test_sanity_check
    ${PROJECT_NAME}
  )

  ament_add_ros_isolated_gtest(test_ground_filter_integration
    test/test_ground_filter_integration.cpp
  )
  target_include_directories(test_ground_filter_integration PRIVATE src)
  target_link_libraries(test_ground_filter_integration
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_ground_filter_integration
    autoware_test_utils
    ament_index_cpp
  )
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  launch
  config
)
