cmake_minimum_required(VERSION 3.8)
project(autoware_marker_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/marker_conversion.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_auto_find_test_dependencies()

  set(test_files
    test/marker_conversion.cpp
  )

  foreach (test_file IN LISTS test_files)
    get_filename_component(test_file_name ${test_file} NAME)
    ament_auto_add_gtest(${test_file_name}_${PROJECT_NAME} ${test_file})
    target_link_libraries(${test_file_name}_${PROJECT_NAME}
      ${PROJECT_NAME}
    )
  endforeach()
endif()

ament_auto_package()
