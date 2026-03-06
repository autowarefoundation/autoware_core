cmake_minimum_required(VERSION 3.14)
project(autoware_command_gate)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  src/autoware_command_gate.cpp
  src/command_gate_mode_builder.cpp
)

target_include_directories(${PROJECT_NAME}_node
  PUBLIC
    include
    src
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "autoware::control::command_gate::AutowareCommandGateNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)

  ament_add_gtest(test_command_gate_mode_builder
    test/unit/test_command_gate_mode_builder.cpp
  )
  target_link_libraries(test_command_gate_mode_builder
    ${PROJECT_NAME}_node
  )
  ament_target_dependencies(test_command_gate_mode_builder
    autoware_adapi_v1_msgs
    autoware_vehicle_msgs
    builtin_interfaces
  )
  set_tests_properties(test_command_gate_mode_builder PROPERTIES LABELS "unit")

  ament_add_gtest(test_ros_integration
    test/integration/test_ros_integration.cpp
  )
  target_link_libraries(test_ros_integration
    ${PROJECT_NAME}_node
  )
  ament_target_dependencies(test_ros_integration
    autoware_adapi_v1_msgs
    autoware_vehicle_msgs
    class_loader
    rclcpp
    rclcpp_components
  )
  set_tests_properties(test_ros_integration PROPERTIES LABELS "integration")

  option(ENABLE_COVERAGE "Enable coverage collection for unit/integration tests" OFF)
  if(ENABLE_COVERAGE)
    foreach(target ${PROJECT_NAME}_node test_command_gate_mode_builder test_ros_integration)
      if(TARGET ${target})
        target_compile_options(${target} PRIVATE -O0 -g --coverage)
        target_link_options(${target} PRIVATE --coverage)
      endif()
    endforeach()

    find_program(LCOV_EXECUTABLE lcov)
    find_program(GENHTML_EXECUTABLE genhtml)
    if(LCOV_EXECUTABLE AND GENHTML_EXECUTABLE)
      add_custom_target(coverage_unit
        DEPENDS test_command_gate_mode_builder
        COMMAND ${CMAKE_COMMAND} -E env AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}
                ${CMAKE_CTEST_COMMAND} -L unit --output-on-failure
        COMMAND ${LCOV_EXECUTABLE} --capture --directory ${CMAKE_BINARY_DIR} --output-file coverage_unit.info
        COMMAND ${LCOV_EXECUTABLE} --extract coverage_unit.info
          '${PROJECT_SOURCE_DIR}/src/*' '${PROJECT_SOURCE_DIR}/include/*'
          --output-file coverage_unit.info
        COMMAND ${GENHTML_EXECUTABLE} -o ${CMAKE_BINARY_DIR}/coverage/unit coverage_unit.info
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating unit test coverage report"
      )

      add_custom_target(coverage_integration
        DEPENDS test_ros_integration
        COMMAND ${CMAKE_COMMAND} -E env AMENT_PREFIX_PATH=${CMAKE_INSTALL_PREFIX}:$ENV{AMENT_PREFIX_PATH}
                ${CMAKE_CTEST_COMMAND} -L integration --output-on-failure
        COMMAND ${LCOV_EXECUTABLE} --capture --directory ${CMAKE_BINARY_DIR} --output-file coverage_integration.info
        COMMAND ${LCOV_EXECUTABLE} --extract coverage_integration.info
          '${PROJECT_SOURCE_DIR}/src/*' '${PROJECT_SOURCE_DIR}/include/*'
          --output-file coverage_integration.info
        COMMAND ${GENHTML_EXECUTABLE} -o ${CMAKE_BINARY_DIR}/coverage/integration coverage_integration.info
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Generating integration test coverage report"
      )
    else()
      message(WARNING "Coverage tools (lcov/genhtml) not found. Coverage targets will be unavailable.")
    endif()
  endif()
endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
    launch
)
