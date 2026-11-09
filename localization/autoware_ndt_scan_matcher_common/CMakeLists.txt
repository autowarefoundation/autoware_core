cmake_minimum_required(VERSION 3.14)
project(autoware_ndt_scan_matcher_common)

# cspell:ignore multigrid

find_package(autoware_cmake REQUIRED)
autoware_package()

# Compile flags for SIMD instructions
# Be careful to change these options, especially when `ndt_omp` implementation is used.
# All packages linked to `ndt_omp` should use the same SIMD instruction set.
# In case mismatched instruction set are used, program causes a crash at its initialization
# because of a misaligned access to the `Eigen` libraries' data structure.
# Keep these options the same as in autoware_ndt_scan_matcher.
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
  # For x86_64 architecture, SIMD instruction set is fixed below versions,
  # because the `ndt_omp` is optimized to these versions.
  add_compile_options(-msse -msse2 -msse3 -msse4 -msse4.1 -msse4.2)
else()
  # For other architecture, like arm64, compile flags are generally prepared by compiler
  # march=native is disabled as default for specific depending pcl libraries
  # or pre-building packages for other computers.
  if(BUILD_WITH_MARCH_NATIVE)
    add_compile_options(-march=native)
  endif()
endif()

find_package(OpenMP)

find_package(PCL REQUIRED COMPONENTS common io registration)
include_directories(${PCL_INCLUDE_DIRS})

# ndt_omp. Packages that depend on this package link it.
ament_auto_add_library(${PROJECT_NAME} SHARED
  src/ndt_omp/multi_voxel_grid_covariance_omp.cpp
  src/ndt_omp/multigrid_ndt_omp.cpp
)
target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

if(OpenMP_CXX_FOUND)
  target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
else()
  message(WARNING "OpenMP not found")
endif()

# The "omp" NDT backend. autoware_ndt_scan_matcher loads it with pluginlib. No other library may
# link it, so it is a plain library: ament_auto_add_library() exports a library, and ament_auto
# then links it into every package that depends on this package.
add_library(${PROJECT_NAME}_omp_backend SHARED
  src/omp_ndt_backend.cpp
)
target_include_directories(${PROJECT_NAME}_omp_backend PRIVATE include)
target_link_libraries(${PROJECT_NAME}_omp_backend ${PROJECT_NAME} ${PCL_LIBRARIES})
ament_target_dependencies(${PROJECT_NAME}_omp_backend pluginlib)
install(TARGETS ${PROJECT_NAME}_omp_backend
  LIBRARY DESTINATION lib
)
pluginlib_export_plugin_description_file(autoware_ndt_scan_matcher plugins.xml)

if(BUILD_TESTING)
  # The "omp" backend against ndt_omp, and its copies.
  ament_auto_add_gtest(test_omp_ndt_backend
    test/test_omp_ndt_backend.cpp
  )
  target_include_directories(test_omp_ndt_backend PRIVATE src)
  target_link_libraries(test_omp_ndt_backend ${PROJECT_NAME}_omp_backend ${PCL_LIBRARIES})
endif()

autoware_ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
)
