cmake_minimum_required(VERSION 3.14)
project(autoware_route_handler)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/route_handler.cpp
)

if(BUILD_TESTING)
  set(fmt_TARGETS fmt::fmt)

  ament_add_ros_isolated_gtest(test_autoware_route_handler
    test/test_main.cpp
    test/test_route_handler.cpp
    test/test_getCenterLinePath.cpp
    test/test_getCenterLinePath_vm_10_spec.cpp
    test/test_getCenterLinePath_vm_10_spec_invalid.cpp)

  ament_target_dependencies(test_autoware_route_handler
    autoware_test_utils
    autoware_pyplot
    fmt
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
  test_data
)
