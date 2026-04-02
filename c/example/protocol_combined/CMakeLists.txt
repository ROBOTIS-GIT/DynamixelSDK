cmake_minimum_required(VERSION 3.10)

# Standalone build for C protocol combined example (uses installed dynamixel_sdk_c).
project(dynamixel_sdk_c_examples_protocol_combined LANGUAGES C)

find_package(dynamixel_sdk_c CONFIG REQUIRED)

add_executable(protocol_combined "${CMAKE_CURRENT_SOURCE_DIR}/protocol_combined.c")
target_link_libraries(protocol_combined PRIVATE dynamixel_sdk_c::dynamixel_sdk_c)

