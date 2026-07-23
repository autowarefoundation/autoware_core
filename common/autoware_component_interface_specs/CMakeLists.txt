cmake_minimum_required(VERSION 3.14)
project(autoware_component_interface_specs)

find_package(autoware_cmake REQUIRED)
autoware_package()

if(BUILD_TESTING)
  # The manifest generator and the concepts test compile real C++20: the
  # `concept X = requires { ... }` syntax in concepts.hpp and tuple iteration. GCC ships
  # standard C++20 concepts only from GCC 10 -- GCC 8/9 have the older Concepts TS
  # (`concept bool`), not this syntax. The Humble RHEL-8 binary build farm job runs GCC 8.5
  # and compiles BUILD_TESTING targets (unlike the Jazzy release jobs, it does not set
  # run_package_tests:false), so these targets would fail to build there. Gate them on a
  # capable compiler so BUILD_TESTING degrades gracefully instead of breaking the build.
  set(AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CXX20 TRUE)
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
    set(AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CXX20 FALSE)
  endif()

  if(AUTOWARE_COMPONENT_INTERFACE_SPECS_HAS_CXX20)
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

    # Sibling of generate_interface_manifest: emits the committed RIHS01 type-hash lockfile.
    # Also C++20 (walks the Specs tuples) and producer-side/CI only, so it lives under the same
    # BUILD_TESTING + C++20-compiler guard. Its hash-emitting body is additionally gated on the
    # Jazzy-only type-hash API at compile time, so on Humble/gcc-11 it builds as a no-op.
    ament_auto_add_executable(generate_type_hashes
      src/generate_type_hashes.cpp
    )
    target_compile_features(generate_type_hashes PRIVATE cxx_std_20)

    # This gtest bundles the C++20-only tests (test_concepts, and test_manifest which drives
    # the generator above) with the plain domain value tests. On GCC 8.5 it is skipped
    # wholesale: the value tests assert compile-time constants identical on every compiler
    # and run on all other jobs, and the C++17 consumer-surface test below still compiles
    # every domain header on the RHEL-8 job.
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
      test/test_type_hashes.cpp
    )
    # test_concepts.cpp exercises the C++20 concepts header.
    target_compile_features(gtest_${PROJECT_NAME} PRIVATE cxx_std_20)
    # COMMITTED_MANIFEST lets test_manifest.cpp diff the generator's output against the file
    # that is actually checked in, which is the copy consumers read.
    target_compile_definitions(gtest_${PROJECT_NAME}
      PRIVATE
        GENERATE_TOOL="$<TARGET_FILE:generate_interface_manifest>"
        COMMITTED_MANIFEST="${CMAKE_CURRENT_SOURCE_DIR}/interface_manifest.json"
        GENERATE_HASHES_TOOL="$<TARGET_FILE:generate_type_hashes>"
        COMMITTED_HASHES="${CMAKE_CURRENT_SOURCE_DIR}/interface_type_hashes.jazzy.lock")
  endif()

  # Consumers of the domain headers get the C++17 that autoware_package() sets, so keep one
  # target on it. Pinned with the property rather than target_compile_features(cxx_std_17),
  # which only asks for *at least* C++17 and would silently follow a raised global default.
  # Deliberately outside the C++20 guard above: this is the surface the RHEL-8 GCC 8.5
  # build farm job still exercises, so the domain headers stay compile-tested there.
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
