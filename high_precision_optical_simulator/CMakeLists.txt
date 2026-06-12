cmake_minimum_required(VERSION 3.25)
project(HighPrecisionOpticalSimulator CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find threads dependency
find_package(Threads REQUIRED)

# Optical positioning core library (HAL + optical)
add_library(optical_core STATIC
  src/hal/LinearAxis.cpp
  src/hal/PressureSensor.cpp
  src/hal/SafetyInterlock.cpp
  src/hal/SpindleMotor.cpp
  src/optical/PositioningFsm.cpp
  src/optical/TelemetryLogger.cpp
  src/optical/ToolpathGenerator.cpp
)

target_include_directories(optical_core PUBLIC
  include
  include/hal
  include/optical
)
target_link_libraries(optical_core PUBLIC Threads::Threads)

if(MSVC)
  target_compile_options(optical_core PRIVATE /W3)
else()
  target_compile_options(optical_core PRIVATE -Wall -Wextra)
endif()

# Tests
add_executable(test_optical
  tests/test_toolpath.cpp
  tests/test_positioning_fsm.cpp
  tests/test_hal.cpp
)
target_link_libraries(test_optical PRIVATE optical_core GTest::gtest_main)
include(GoogleTest)
gtest_discover_tests(test_optical)
