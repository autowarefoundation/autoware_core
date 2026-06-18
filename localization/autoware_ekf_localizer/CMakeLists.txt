cmake_minimum_required(VERSION 3.14)
project(autoware_ekf_localizer)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(autoware_agnocast_wrapper REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
    src/
)

ament_auto_find_build_dependencies()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ekf_localizer.cpp
  src/hyper_parameters.cpp
  src/covariance.cpp
  src/diagnostics.cpp
  src/mahalanobis.cpp
  src/measurement.cpp
  src/state_transition.cpp
  src/warning_message.cpp
  src/ekf_module.cpp
)

ament_target_dependencies(${PROJECT_NAME} autoware_agnocast_wrapper)

autoware_agnocast_wrapper_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ekf_localizer::EKFLocalizer"
  EXECUTABLE ${PROJECT_NAME}_node
  AGNOCAST_EXECUTOR AgnocastOnlyCallbackIsolatedExecutor
  ROS2_EXECUTOR SingleThreadedExecutor
)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

function(add_testcase filepath)
  get_filename_component(filename ${filepath} NAME)
  string(REGEX REPLACE ".cpp" "" test_name ${filename})

  ament_add_gtest(${test_name} ${filepath})
  target_link_libraries("${test_name}" ${PROJECT_NAME})
  ament_target_dependencies(${test_name} ${${PROJECT_NAME}_FOUND_BUILD_DEPENDS})
  ament_target_dependencies(${test_name} autoware_agnocast_wrapper)
  autoware_agnocast_wrapper_setup(${test_name})
endfunction()

if(BUILD_TESTING)
  add_launch_test(
    test/test_ekf_localizer_launch.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_ekf_localizer_mahalanobis.py
    TIMEOUT "30"
  )
  find_package(ament_cmake_gtest REQUIRED)

  file(GLOB_RECURSE TEST_FILES test/*.cpp)

  if(DEFINED ENV{ENABLE_AGNOCAST} AND "$ENV{ENABLE_AGNOCAST}" STREQUAL "1")
    list(FILTER TEST_FILES EXCLUDE REGEX "test_diagnostics(_topic)?\\.cpp$")
  endif()

  foreach(filepath ${TEST_FILES})
    add_testcase(${filepath})
  endforeach()
endif()


# if(BUILD_TESTING)
#   find_package(ament_cmake_ros REQUIRED)
#   ament_add_ros_isolated_gtest(ekf_localizer-test test/test_ekf_localizer.test
#     test/src/test_ekf_localizer.cpp
#     src/ekf_localizer.cpp
#     src/kalman_filter/kalman_filter.cpp
#     src/kalman_filter/time_delay_kalman_filter.cpp
#   )
#   target_include_directories(ekf_localizer-test
#     PRIVATE
#     include
#   )
#   ament_target_dependencies(ekf_localizer-test geometry_msgs rclcpp tf2 tf2_ros)
# endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE
  config
  launch
)
