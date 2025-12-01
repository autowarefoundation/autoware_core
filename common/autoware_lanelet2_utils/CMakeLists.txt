cmake_minimum_required(VERSION 3.8)
project(autoware_lanelet2_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kind.cpp
  src/topology.cpp
  src/stop_line.cpp
  src/intersection.cpp
  src/conversion.cpp
  src/geometry.cpp
  src/hatched_road_markings.cpp
  src/lane_sequence.cpp
  src/nn_search.cpp
  src/map_handler.cpp
  src/route_manager.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  find_package(autoware_test_utils REQUIRED)
  find_package(yaml-cpp REQUIRED)
  ament_auto_find_test_dependencies()

  set(test_files
    test/kind.cpp
    test/topology.cpp
    test/intersection.cpp
    test/stop_line.cpp
    test/geometry.cpp
    test/hatched_road_marking.cpp
    test/conversion.cpp
    test/lane_sequence.cpp
    test/nn_search.cpp
    test/map_handler.cpp
    test/route_manager.cpp
  )

  foreach (test_file IN LISTS test_files)
    get_filename_component(test_file_name ${test_file} NAME)
    ament_auto_add_gtest(${test_file_name}_${PROJECT_NAME} ${test_file})
    target_link_libraries(${test_file_name}_${PROJECT_NAME}
      ${PROJECT_NAME}
      yaml-cpp
    )
  endforeach()
endif()

autoware_ament_auto_package(INSTALL_TO_SHARE
  sample_map
  test_data
)

install(PROGRAMS
  scripts/lanelet_anonymizer.py
  scripts/lanelet_id_aligner.py
  scripts/test_case_generator.py
  DESTINATION lib/${PROJECT_NAME}
)
