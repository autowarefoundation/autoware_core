cmake_minimum_required(VERSION 3.14)
project(autoware_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(autoware_agnocast_wrapper REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_initializer_core.cpp
  src/gnss_module.cpp
  src/localization_module.cpp
  src/stop_check_module.cpp
  src/pose_error_check_module.cpp
  src/localization_trigger_module.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

# PoseInitializer derives from autoware::agnocast_wrapper::Node, which requires an AgnocastOnly
# executor, so it runs as a standalone executable rather than inside a component container.
autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_initializer::PoseInitializer"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyMultiThreadedExecutor
)

if(BUILD_TESTING)
  function(add_testcase filepath)
    get_filename_component(filename ${filepath} NAME)
    string(REGEX REPLACE ".cpp" "" test_name ${filename})
    ament_add_gmock(${test_name} ${filepath})
    target_include_directories(${test_name} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  endfunction()

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  add_testcase(test/test_copy_vector_to_array.cpp)
  add_testcase(test/test_localization_util.cpp)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
