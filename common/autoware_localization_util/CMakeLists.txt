cmake_minimum_required(VERSION 3.14)
project(autoware_localization_util)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/util_func.cpp
  src/smart_pose_buffer.cpp
  src/tree_structured_parzen_estimator.cpp
  src/covariance_ellipse.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_auto_add_gtest(test_smart_pose_buffer
    test/test_smart_pose_buffer.cpp
    src/smart_pose_buffer.cpp
  )
  target_link_libraries(test_smart_pose_buffer ${PROJECT_NAME})

  ament_auto_add_gtest(test_tpe
    test/test_tpe.cpp
    src/tree_structured_parzen_estimator.cpp
  )
  target_link_libraries(test_tpe ${PROJECT_NAME})

  ament_auto_add_gtest(test_covariance_ellipse
    test/test_covariance_ellipse.cpp
    src/covariance_ellipse.cpp
  )
  target_link_libraries(test_covariance_ellipse ${PROJECT_NAME})

  ament_auto_add_gtest(test_util_func
    test/test_util_func.cpp
    src/util_func.cpp
  )
  target_link_libraries(test_util_func ${PROJECT_NAME})

endif()

autoware_ament_auto_package(
  INSTALL_TO_SHARE
)

install(DIRECTORY include/ DESTINATION include)
