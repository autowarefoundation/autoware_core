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

  # Examples
  set(example_files
    examples/example_geometry.cpp
    examples/example_conversion.cpp
    examples/example_kind.cpp
    examples/example_hatched_road_markings.cpp
    examples/example_topology.cpp
    examples/example_intersection.cpp
    examples/example_lane_sequence.cpp
    examples/example_nn_search.cpp
    examples/example_map_handler.cpp
    examples/example_route_manager.cpp
  )

  find_package(ament_index_cpp REQUIRED)
  find_package(python_cmake_module REQUIRED)
  find_package(
    Python3
    COMPONENTS Interpreter Development
    REQUIRED)
  find_package(range-v3 REQUIRED)
  find_package(pybind11_vendor REQUIRED)
  find_package(pybind11 REQUIRED)
  find_package(yaml-cpp REQUIRED)
  foreach(example_file ${example_files})
    get_filename_component(example_name ${example_file} NAME_WE)
    ament_auto_add_executable(${example_name}
      ${example_file}
    )
    ament_target_dependencies(${example_name} ament_index_cpp)
    target_link_libraries(${example_name}
      autoware_lanelet2_utils
      range-v3::range-v3
      Python3::Python
      pybind11::embed
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
