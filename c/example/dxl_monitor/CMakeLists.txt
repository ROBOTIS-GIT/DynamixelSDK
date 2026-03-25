cmake_minimum_required(VERSION 3.10)

# Standalone build for C dxl_monitor example (uses installed dynamixel_sdk_c).
project(dynamixel_sdk_c_examples_dxl_monitor LANGUAGES C)

find_package(dynamixel_sdk_c CONFIG REQUIRED)

add_executable(dxl_monitor "${CMAKE_CURRENT_SOURCE_DIR}/dxl_monitor.c")
target_link_libraries(dxl_monitor PRIVATE dynamixel_sdk_c::dynamixel_sdk_c)

