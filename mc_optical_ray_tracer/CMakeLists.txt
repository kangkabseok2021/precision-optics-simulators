cmake_minimum_required(VERSION 3.25)
project(MCOpticalRayTracer CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(BUILD_WITH_CUDA "Enable CUDA backend" OFF)

find_package(Threads REQUIRED)

# Geometry library
add_library(geometry STATIC
  src/geometry/Triangle.cpp
  src/geometry/AABB.cpp
  src/geometry/BVH.cpp
)
target_include_directories(geometry PUBLIC include)

# Optics library
add_library(optics STATIC
  src/optics/Fresnel.cpp
  src/optics/BRDF.cpp
)
target_include_directories(optics PUBLIC include)
target_link_libraries(optics PUBLIC geometry)

# Tracer library
add_library(tracer STATIC
  src/tracer/Camera.cpp
  src/tracer/Scene.cpp
  src/tracer/PathTracer.cpp
)
target_include_directories(tracer PUBLIC include)
target_link_libraries(tracer PUBLIC geometry optics)

# Lighting library
add_library(lighting STATIC
  src/lighting/LuminanceMap.cpp
  src/lighting/Photometry.cpp
)
target_include_directories(lighting PUBLIC include)

# Main executable
add_executable(ray_tracer src/main.cpp)
target_link_libraries(ray_tracer PRIVATE tracer lighting)

# Tests
include(GoogleTest)

add_executable(test_ray_bvh tests/test_ray_bvh.cpp)
target_link_libraries(test_ray_bvh PRIVATE geometry GTest::gtest_main)
gtest_discover_tests(test_ray_bvh)

add_executable(test_optics tests/test_optics.cpp)
target_link_libraries(test_optics PRIVATE optics GTest::gtest_main)
gtest_discover_tests(test_optics)

add_executable(test_path_tracer tests/test_path_tracer.cpp)
target_link_libraries(test_path_tracer PRIVATE tracer lighting GTest::gtest_main)
gtest_discover_tests(test_path_tracer)
