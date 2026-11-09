cmake_minimum_required(VERSION 3.14)
project(autoware_ndt_scan_matcher_common)

find_package(autoware_cmake REQUIRED)
autoware_package()

# This package has headers only. They include Eigen and PCL, so the packages that depend on this
# package find both. Export all of PCL: a package that first finds only some components of PCL can
# miss the targets of the other components when it finds PCL again (for example, through pcl_ros).
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
ament_export_dependencies(eigen3_cmake_module Eigen3 PCL)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(class_loader REQUIRED)

  # The same fake backend as several plugin libraries: once with the Eigen configuration of the
  # test, and once for each definition that can change the layout of the Eigen types or the heap
  # allocator of Eigen, as a backend that was built with other compiler options.
  set(fake_backend_definitions_same "")
  set(fake_backend_definitions_align_0 EIGEN_MAX_STATIC_ALIGN_BYTES=0)
  set(fake_backend_definitions_align_16 EIGEN_MAX_STATIC_ALIGN_BYTES=16)
  set(fake_backend_definitions_align_32 EIGEN_MAX_STATIC_ALIGN_BYTES=32)
  set(fake_backend_definitions_align_64 EIGEN_MAX_STATIC_ALIGN_BYTES=64)
  set(fake_backend_definitions_allocator_eigen EIGEN_MALLOC_ALREADY_ALIGNED=0)
  set(fake_backend_definitions_allocator_std EIGEN_MALLOC_ALREADY_ALIGNED=1)

  ament_add_gtest(test_ndt_backend_abi
    test/test_ndt_backend_abi.cpp
  )
  target_include_directories(test_ndt_backend_abi PRIVATE include)
  target_include_directories(test_ndt_backend_abi SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(test_ndt_backend_abi
    Eigen3::Eigen class_loader::class_loader ${CMAKE_DL_LIBS})

  foreach(variant
    same align_0 align_16 align_32 align_64 allocator_eigen allocator_std)
    add_library(fake_ndt_backend_${variant} MODULE test/fake_ndt_backend.cpp)
    target_include_directories(fake_ndt_backend_${variant} PRIVATE include)
    target_include_directories(fake_ndt_backend_${variant} SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
    target_link_libraries(fake_ndt_backend_${variant} Eigen3::Eigen class_loader::class_loader)
    target_compile_definitions(fake_ndt_backend_${variant} PRIVATE
      ${fake_backend_definitions_${variant}})
    # $<1:...> keeps a multi-config generator from adding a directory for each configuration.
    set_target_properties(fake_ndt_backend_${variant} PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY $<1:${CMAKE_CURRENT_BINARY_DIR}/fake_ndt_backends>)
    add_dependencies(test_ndt_backend_abi fake_ndt_backend_${variant})
  endforeach()
  target_compile_definitions(test_ndt_backend_abi PRIVATE
    FAKE_BACKEND_DIRECTORY="${CMAKE_CURRENT_BINARY_DIR}/fake_ndt_backends")
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
