cmake_minimum_required(VERSION 3.10)

# Standalone build for C++ DXL Monitor example (uses installed dynamixel_sdk).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(dynamixel_sdk_cpp_examples_dxl_monitor LANGUAGES CXX)
endif()

find_package(dynamixel_sdk CONFIG REQUIRED)

add_executable(dxl_monitor "${CMAKE_CURRENT_SOURCE_DIR}/dxl_monitor.cpp")
target_link_libraries(dxl_monitor PRIVATE dynamixel_sdk::dynamixel_sdk)

