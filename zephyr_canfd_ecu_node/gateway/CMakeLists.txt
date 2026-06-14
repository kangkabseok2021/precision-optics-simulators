cmake_minimum_required(VERSION 3.25)
project(EcuGateway CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)
find_package(OpenSSL)
find_package(PahoMqttCpp QUIET)

# Gracefully skip compilation if OpenSSL or PahoMqttCpp is not installed on host
if(NOT OpenSSL_FOUND OR NOT PahoMqttCpp_FOUND)
    message(WARNING "zephyr_canfd_ecu_node gateway dependencies (OpenSSL, PahoMqttCpp) not found. Gateway local build skipped.")
    return()
endif()

include(FetchContent)
FetchContent_Declare(
  cpp-httplib
  URL https://github.com/yhirose/cpp-httplib/archive/v0.15.3.tar.gz
)
FetchContent_MakeAvailable(cpp-httplib)

add_library(gateway_lib STATIC
    src/frame_codec.cpp
    src/SocketCanFd.cpp
    src/MqttPublisher.cpp
    src/HttpsApi.cpp
)

target_include_directories(gateway_lib PUBLIC include)
target_link_libraries(gateway_lib PUBLIC
    OpenSSL::SSL
    OpenSSL::Crypto
    PahoMqttCpp::paho-mqtt-cpp
    cpp-httplib::cpp-httplib
    Threads::Threads
)

add_executable(ecu_gateway src/main.cpp)
target_link_libraries(ecu_gateway PRIVATE gateway_lib)

if(gtest_resolved OR TARGET gtest)
    add_executable(test_gateway tests/test_gateway.cpp)
    target_link_libraries(test_gateway PRIVATE gateway_lib gtest_main)
    add_test(NAME test_gateway COMMAND test_gateway)
endif()
