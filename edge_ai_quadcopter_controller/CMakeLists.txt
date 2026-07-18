cmake_minimum_required(VERSION 3.20)
project(edge_ai_quadcopter_controller CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_TESTS "Build the GoogleTest suite (host only)" ON)
option(BUILD_BENCHMARK "Build the Google Benchmark harness (host only)" OFF)

# Core library: model_data.cc is the only non-header-only translation
# unit — PidController, AttitudePid, MotorMixer, and InferenceEngine are
# header-only so the same code path compiles unmodified for the host
# and for the ARM cross target below.
add_library(eaq_core STATIC src/model_data.cc)
target_include_directories(eaq_core PUBLIC include)
target_compile_features(eaq_core PUBLIC cxx_std_17)

if(CMAKE_CROSSCOMPILING)
  # Cortex-M7 firmware smoke target — proves the full algorithmic core
  # (PID, motor mixer, inference engine) compiles under the ARM
  # cross toolchain with the hard-float ABI. A static library, not an
  # executable: no linker script or startup code is needed to prove
  # the code cross-compiles and to report arm-none-eabi-size output.
  add_library(eaq_arm_smoke STATIC src/arm_smoke.cpp)
  target_link_libraries(eaq_arm_smoke PUBLIC eaq_core)
else()
  add_executable(main_sim src/main_sim.cpp)
  target_link_libraries(main_sim PRIVATE eaq_core)

  if(BUILD_TESTS)
    if(NOT TARGET gtest_main)
      include(FetchContent)
      FetchContent_Declare(
        googletest
        URL https://github.com/google/googletest/archive/v1.14.0.tar.gz
      )
      set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googletest)
    endif()

    enable_testing()
    add_executable(eaq_tests
      tests/test_pid_controller.cpp
      tests/test_motor_mixer.cpp
      tests/test_inference_engine.cpp
    )
    target_link_libraries(eaq_tests PRIVATE eaq_core gtest_main)
    include(GoogleTest)
    gtest_discover_tests(eaq_tests)
  endif()

  if(BUILD_BENCHMARK)
    if(NOT TARGET benchmark::benchmark)
      include(FetchContent)
      FetchContent_Declare(
        googlebenchmark
        URL https://github.com/google/benchmark/archive/v1.8.3.tar.gz
      )
      set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
      set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
      set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
      FetchContent_MakeAvailable(googlebenchmark)
    endif()

    add_executable(eaq_benchmark src/benchmark_main.cpp)
    target_link_libraries(eaq_benchmark PRIVATE eaq_core benchmark::benchmark)
  endif()
endif()
