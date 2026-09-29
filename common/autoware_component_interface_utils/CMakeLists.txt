cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_utils)

find_package(autoware_cmake REQUIRED)
autoware_package()

# ament_auto_find_build_dependencies() (invoked by autoware_package() above) calls
# find_package() using the exact package.xml dependency string, which is the
# rosdep key "nlohmann-json-dev" here. That key does not match the CMake package
# name the system install actually exports ("nlohmann_json"), so it must be
# found explicitly.
find_package(nlohmann_json REQUIRED)

include_directories(
  include
  SYSTEM
  ${rclcpp_INCLUDE_DIRS}
  ${autoware_adapi_v1_msgs_INCLUDE_DIRS}
)

if(BUILD_TESTING)
  ament_add_ros_isolated_gtest(test_component_interface_utils
    test/test_component_interface_utils.cpp
  )

  target_include_directories(test_component_interface_utils
    PRIVATE include
  )

  ament_target_dependencies(test_component_interface_utils
    rclcpp
    rclcpp_components
    autoware_adapi_v1_msgs
    autoware_component_interface_specs
  )

  ament_add_ros_isolated_gtest(test_component_interface_registration
    test/test_registration.cpp
  )

  target_include_directories(test_component_interface_registration
    PRIVATE include
  )

  ament_target_dependencies(test_component_interface_registration
    rclcpp
    rclcpp_components
    autoware_adapi_v1_msgs
    autoware_component_interface_specs
  )

  # Pure mapping logic over InterfaceRecord -- no rclcpp::init(), no Node, no ROS
  # graph -- so it does not need the isolated-domain-ID treatment the two
  # targets above require.
  ament_add_gtest(test_manifest_json
    test/test_manifest_json.cpp
  )

  target_include_directories(test_manifest_json
    PRIVATE include
  )

  ament_target_dependencies(test_manifest_json
    rclcpp
    rclcpp_components
    autoware_adapi_v1_msgs
    autoware_component_interface_specs
  )

  target_link_libraries(test_manifest_json
    nlohmann_json::nlohmann_json
  )

  # Exercises the testing/manifest_drift.hpp helper end to end: a real Node +
  # NodeAdaptor (hence the ROS graph/isolated-domain-ID treatment, same as
  # test_component_interface_registration above) pinned against fragment files
  # committed under test/data/. Those fragments are test data only, so they are
  # neither installed nor part of the public API.
  ament_add_ros_isolated_gtest(test_manifest_drift
    test/test_manifest_drift.cpp
  )

  target_include_directories(test_manifest_drift
    PRIVATE include
  )

  ament_target_dependencies(test_manifest_drift
    rclcpp
    rclcpp_components
    autoware_adapi_v1_msgs
    autoware_component_interface_specs
  )

  target_link_libraries(test_manifest_drift
    nlohmann_json::nlohmann_json
  )

  # MATCHING_FRAGMENT/DIVERGENT_FRAGMENT let test_manifest_drift.cpp read the
  # fragment files without hardcoding a path relative to the working
  # directory the test happens to run from -- the same idiom
  # autoware_component_interface_specs uses for COMMITTED_MANIFEST.
  target_compile_definitions(test_manifest_drift
    PRIVATE
      MATCHING_FRAGMENT="${CMAKE_CURRENT_SOURCE_DIR}/test/data/matching_fragment.json"
      DIVERGENT_FRAGMENT="${CMAKE_CURRENT_SOURCE_DIR}/test/data/divergent_fragment.json")
endif()

# nlohmann-json-dev never lands in ${PROJECT_NAME}_FOUND_BUILD_DEPENDS (see the
# find_package() comment above), so ament_auto_package() would not export it on
# its own. manifest_json.hpp is a publicly installed header that requires
# nlohmann::json, so downstream C++ consumers of this package need the
# dependency exported explicitly.
ament_export_dependencies(nlohmann_json)

ament_auto_package()
