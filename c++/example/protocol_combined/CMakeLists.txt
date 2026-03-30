cmake_minimum_required(VERSION 3.10)

# Standalone build for C++ Protocol Combined example (uses installed dynamixel_sdk).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(dynamixel_sdk_cpp_examples_protocol_combined LANGUAGES CXX)
endif()

find_package(dynamixel_sdk CONFIG REQUIRED)

add_executable(protocol_combined
  "${CMAKE_CURRENT_SOURCE_DIR}/protocol_combined.cpp"
)
target_link_libraries(protocol_combined PRIVATE dynamixel_sdk::dynamixel_sdk)

