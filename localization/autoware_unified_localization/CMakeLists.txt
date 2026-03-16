cmake_minimum_required(VERSION 3.14)
project(autoware_unified_localization)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)

# ROS-free Core library (EKF + stop_filter + twist2accel)
add_library(unified_localization_core SHARED
  src/core/ekf_core.cpp
  src/core/fusion_pipeline.cpp
  src/core/measurement_core.cpp
  src/core/state_transition_core.cpp
  src/core/stop_filter_core.cpp
  src/core/twist2accel_core.cpp
)
target_include_directories(unified_localization_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(unified_localization_core Eigen3 autoware_kalman_filter)
target_link_libraries(unified_localization_core Eigen3::Eigen)

install(TARGETS unified_localization_core
  EXPORT unified_localization_core
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
  PATTERN "*.hpp"
)

# Adapter: one node that subscribes pose/twist, calls Core, publishes odom and acceleration
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node unified_localization_core)
ament_target_dependencies(
  localization_node
  rclcpp
  geometry_msgs
  nav_msgs
  std_srvs
  tf2_ros
  autoware_adapi_v1_msgs
  autoware_component_interface_specs
  autoware_internal_localization_msgs
  diagnostic_msgs
)
install(TARGETS localization_node DESTINATION lib/${PROJECT_NAME})

# Install config and launch for the node
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_targets(unified_localization_core HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 autoware_kalman_filter)

if(BUILD_TESTING)
  set(CMAKE_THREADS_PREFER_PTHREAD_LIB ON)
  find_package(Threads QUIET)
  # Fallback: build tests when libpthread is found even if FindThreads failed (e.g. cache)
  if(NOT Threads_FOUND AND CMAKE_SYSTEM_NAME STREQUAL "Linux")
    find_library(PTHREAD_LIBRARY pthread)
    if(PTHREAD_LIBRARY)
      add_library(Threads::Threads INTERFACE IMPORTED)
      set_target_properties(Threads::Threads PROPERTIES INTERFACE_LINK_LIBRARIES "${PTHREAD_LIBRARY}")
      set(Threads_FOUND TRUE)
      message(STATUS "Using pthread directly (FindThreads fallback)")
    endif()
  endif()
  if(Threads_FOUND)
    find_package(ament_cmake_gtest REQUIRED)
    ament_add_gtest(test_fusion_pipeline test/test_fusion_pipeline.cpp)
    target_link_libraries(test_fusion_pipeline unified_localization_core Threads::Threads)
    ament_target_dependencies(test_fusion_pipeline Eigen3 autoware_kalman_filter)
    # Install test RUNTIME so ros2 run can find it
    install(TARGETS test_fusion_pipeline RUNTIME DESTINATION lib/${PROJECT_NAME})
  else()
    message(STATUS "Threads not found; skipping test_fusion_pipeline (run with -DBUILD_TESTING=OFF to avoid)")
  endif()

  add_launch_test(
    test/test_unified_localization_launch.py
    TIMEOUT "30"
  )
  add_launch_test(
    test/test_gnss_only_auto_launch.py
    TIMEOUT "30"
  )
endif()

autoware_ament_auto_package()
