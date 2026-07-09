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
    test/test_service_qos.cpp
  )
  # test_concepts.cpp exercises the C++20 concepts header.
  target_compile_features(gtest_${PROJECT_NAME} PRIVATE cxx_std_20)
  # COMMITTED_MANIFEST lets test_manifest.cpp diff the generator's output against the file
  # that is actually checked in, which is the copy consumers read.
  target_compile_definitions(gtest_${PROJECT_NAME}
    PRIVATE
      GENERATE_TOOL="$<TARGET_FILE:generate_interface_manifest>"
      COMMITTED_MANIFEST="${CMAKE_CURRENT_SOURCE_DIR}/interface_manifest.json")

  # Consumers of the domain headers get the C++17 that autoware_package() sets, so keep one
  # target on it. Pinned with the property rather than target_compile_features(cxx_std_17),
  # which only asks for *at least* C++17 and would silently follow a raised global default.
  ament_auto_add_gtest(gtest_${PROJECT_NAME}_cxx17
    test/gtest_main.cpp
    test/test_cxx17_compat.cpp
  )
  set_target_properties(gtest_${PROJECT_NAME}_cxx17 PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF)
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
