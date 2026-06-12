cmake_minimum_required(VERSION 3.25)
project(PrecisionOpticsSimulators CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Enable testing globally
enable_testing()

# Fetch GoogleTest for subproject tests
include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/v1.14.0.tar.gz
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add subprojects
add_subdirectory(high_precision_optical_simulator)

# Add the planned ray tracer only if its directory contains a CMakeLists.txt
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/mc_optical_ray_tracer/CMakeLists.txt")
    add_subdirectory(mc_optical_ray_tracer)
endif()
