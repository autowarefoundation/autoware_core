cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_specs)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  # Build-time tool that emits the committed interface_manifest.json. It walks the
  # per-domain Specs tuples, so it needs C++20 (concepts + tuple iteration). It is
  # a producer-side/CI tool only (the manifest freshness test drives it), so it is
  # gated behind BUILD_TESTING: this keeps the C++20 build off the default consumer
  # dependency build, so it can never break the header-only package for the C++17
  # consumers that include the domain headers.
  ament_auto_add_executable(generate_interface_manifest
    src/generate_interface_manifest.cpp
  )
  target_compile_features(generate_interface_manifest PRIVATE cxx_std_20)

  ament_auto_add_gtest(gtest_${PROJECT_NAME}
    test/gtest_main.cpp
    test/test_planning.cpp
    test/test_control.cpp
    test/test_localization.cpp
    test/test_map.cpp
    test/test_perception.cpp
    test/test_system.cpp
    test/test_vehicle.cpp
    test/test_version.cpp
    test/test_concepts.cpp
    test/test_manifest.cpp
  )
  # test_concepts.cpp exercises the C++20 concepts header.
  target_compile_features(gtest_${PROJECT_NAME} PRIVATE cxx_std_20)
  target_compile_definitions(gtest_${PROJECT_NAME}
    PRIVATE GENERATE_TOOL="$<TARGET_FILE:generate_interface_manifest>")
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
