cmake_minimum_required(VERSION 3.14)
project(autoware_route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/route_handler.cpp
)

if(BUILD_TESTING)
  set(fmt_TARGETS fmt::fmt)

  find_package(
    Python3
    COMPONENTS Interpreter Development
    REQUIRED)
  list(APPEND ${PROJECT_NAME}_FOUND_TEST_DEPENDS
    Python3
  )
  set(Python3_TARGETS Python3::Python)
  # Humble's yaml-cpp 0.7 has no yaml-cpp::yaml-cpp target
  if(TARGET yaml-cpp::yaml-cpp)
    set(yaml-cpp_TARGETS yaml-cpp::yaml-cpp)
  endif()

  ament_add_ros_isolated_gtest(test_autoware_route_handler
    test/test_main.cpp
    test/test_route_handler.cpp
    test/test_allow_area_route_handler.cpp
    test/test_getCenterLinePath.cpp
    test/test_getCenterLinePath_vm_10_spec.cpp
    test/test_getCenterLinePath_vm_10_spec_invalid.cpp)

  ament_libraries_deduplicate(deduplicated_libraries
    ${autoware_lanelet2_utils_LIBRARIES}
    ${autoware_test_utils_LIBRARIES}
    ${autoware_pyplot_LIBRARIES}
  )
  target_link_libraries(test_autoware_route_handler
    ${deduplicated_libraries}
    ${fmt_TARGETS}
    ${yaml-cpp_TARGETS}
    ${Python3_TARGETS}
  )
  target_include_directories(test_autoware_route_handler PRIVATE
    ${autoware_lanelet2_utils_INCLUDE_DIRS}
    ${autoware_test_utils_INCLUDE_DIRS}
    ${autoware_pyplot_INCLUDE_DIRS}
  )

  target_compile_definitions(test_${PROJECT_NAME}
    PUBLIC PLOT
  )

  target_link_libraries(test_autoware_route_handler
    ${PROJECT_NAME}
  )

endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  test_route
)
