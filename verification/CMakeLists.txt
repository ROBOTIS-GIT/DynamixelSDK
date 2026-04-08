cmake_minimum_required(VERSION 3.10)

project(dynamixel_sdk_verification LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(dynamixel_sdk REQUIRED CONFIG)

add_executable(dxl_sdk_verifier
  sdk_verifier.cpp
)

target_compile_definitions(dxl_sdk_verifier PRIVATE
  VERIFICATION_CONTROL_TABLE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/../control_table"
)

target_link_libraries(dxl_sdk_verifier PRIVATE dynamixel_sdk::dynamixel_sdk)
