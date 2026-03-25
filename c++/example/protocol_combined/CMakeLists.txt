cmake_minimum_required(VERSION 3.10)

# Standalone build for C++ Protocol Combined example (uses installed dynamixel_sdk).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(dynamixel_sdk_cpp_examples_protocol_combined LANGUAGES CXX)
endif()

find_package(dynamixel_sdk CONFIG REQUIRED)

set(_dxl_sdk_target dynamixel_sdk::dynamixel_sdk)

set(_dxl_extra_includes "")
get_target_property(_dxl_iface_includes ${_dxl_sdk_target} INTERFACE_INCLUDE_DIRECTORIES)
foreach(_dxl_inc IN LISTS _dxl_iface_includes)
  list(APPEND _dxl_extra_includes "${_dxl_inc}/dynamixel_sdk")
endforeach()

add_executable(protocol_combined
  "${CMAKE_CURRENT_SOURCE_DIR}/protocol_combined.cpp"
)
target_include_directories(protocol_combined PRIVATE ${_dxl_extra_includes})
target_link_libraries(protocol_combined PRIVATE ${_dxl_sdk_target})

