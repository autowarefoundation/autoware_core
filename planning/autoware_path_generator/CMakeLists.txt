cmake_minimum_required(VERSION 3.14)
project(autoware_path_generator)

find_package(autoware_cmake REQUIRED)
autoware_package()

generate_parameter_library(path_generator_parameters
  param/path_generator_parameters.yaml
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

target_link_libraries(${PROJECT_NAME}
  path_generator_parameters
)

target_compile_options(${PROJECT_NAME} PUBLIC
  -Wno-error=deprecated-declarations
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::path_generator::PathGenerator"
  EXECUTABLE path_generator_node
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME}
    test/test_dense_centerline.cpp
    test/test_goal_connection.cpp
    test/test_path_generator_node_interface.cpp
    test/test_lanelet.cpp
    test/test_turn_signal.cpp
    test/test_path_cut.cpp
  )
  target_link_libraries(test_${PROJECT_NAME}
    ${PROJECT_NAME}
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
    config
    test_route
)
