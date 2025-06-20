cmake_minimum_required(VERSION 3.14)
project(autoware_kalman_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

# Check if AVX2 is supported
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)

if(COMPILER_SUPPORTS_AVX2)
# cspell:ignore mfma
    add_compile_options(-mavx2 -mfma -O3)  # Enable AVX2 & FMA
endif()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  SYSTEM
    ${EIGEN3_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kalman_filter.cpp
  src/time_delay_kalman_filter.cpp
  include/autoware/kalman_filter/kalman_filter.hpp
  include/autoware/kalman_filter/time_delay_kalman_filter.hpp
)

if(BUILD_TESTING)
  file(GLOB_RECURSE test_files test/*.cpp)
  ament_add_ros_isolated_gtest(test_${PROJECT_NAME} ${test_files})

  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME})
endif()

ament_auto_package()
