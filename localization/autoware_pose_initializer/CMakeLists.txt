cmake_minimum_required(VERSION 3.14)
project(autoware_pose_initializer)

find_package(autoware_cmake REQUIRED)
autoware_package()
find_package(autoware_agnocast_wrapper REQUIRED)
find_package(autoware_map_height_fitter REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/pose_initializer.cpp
  src/pose_initializer_node.cpp
  src/utils/gnss_module.cpp
  src/utils/localization_module.cpp
  src/utils/stop_check_module.cpp
  src/utils/pose_error_check_module.cpp
  src/utils/localization_trigger_module.cpp
)
ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper autoware_map_height_fitter)
autoware_agnocast_wrapper_setup(${PROJECT_NAME})

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::pose_initializer::PoseInitializerNode"
  EXECUTABLE ${PROJECT_NAME}_node
  ROS2_EXECUTOR MultiThreadedExecutor
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
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

  # Agnocast deliberately aborts the process when a Publisher or a Subscription is created
  # without libagnocast_heaphook.so in LD_PRELOAD (validate_ld_preload() in agnocast_utils.cpp).
  # We do not want the tests to depend on the heaphook or on the kernel module, so the tests
  # that create publishers/subscriptions are the ones skipped at ENABLE_AGNOCAST=1.
  if(NOT (DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1"))
    ament_add_gtest(test_pose_initializer_node_integration test/test_pose_initializer_node_integration.cpp)
    target_include_directories(test_pose_initializer_node_integration PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(test_pose_initializer_node_integration ${PROJECT_NAME})
    ament_target_dependencies(test_pose_initializer_node_integration ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
    ament_target_dependencies(test_pose_initializer_node_integration autoware_agnocast_wrapper autoware_map_height_fitter)
    autoware_agnocast_wrapper_setup(test_pose_initializer_node_integration)
  endif()
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  launch
  config
)
