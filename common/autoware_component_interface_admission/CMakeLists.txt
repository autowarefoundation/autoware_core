cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_admission)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(nlohmann_json REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/manifest_json.cpp
)
target_link_libraries(${PROJECT_NAME} nlohmann_json::nlohmann_json)

# Deploy-time admission gate CLI (ros2 run autoware_component_interface_admission manifest_admit).
ament_auto_add_executable(manifest_admit src/manifest_admit.cpp)
target_link_libraries(manifest_admit ${PROJECT_NAME})

if(BUILD_TESTING)
  ament_auto_add_gtest(test_admission_rule test/test_admission_rule.cpp)
  target_link_libraries(test_admission_rule ${PROJECT_NAME})

  ament_auto_add_gtest(test_manifest_json test/test_manifest_json.cpp)
  target_link_libraries(test_manifest_json ${PROJECT_NAME})

  ament_auto_add_gtest(test_manifest_admit test/test_manifest_admit.cpp)
  target_link_libraries(test_manifest_admit ${PROJECT_NAME})
endif()

ament_auto_package()
