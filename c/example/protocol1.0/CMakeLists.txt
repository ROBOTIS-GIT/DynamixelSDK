cmake_minimum_required(VERSION 3.10)

# Standalone build for C Protocol 1.0 examples (uses installed dynamixel_sdk_c).
project(dynamixel_sdk_c_examples_protocol1 LANGUAGES C)

find_package(dynamixel_sdk_c CONFIG REQUIRED)

set(_dxl_protocol1_examples
  sync_write
  read_write
  reset
  multi_port
  ping
  bulk_read
)

foreach(_dxl_example IN LISTS _dxl_protocol1_examples)
  set(_dxl_src "${CMAKE_CURRENT_SOURCE_DIR}/${_dxl_example}/${_dxl_example}.c")
  if(NOT EXISTS "${_dxl_src}")
    message(FATAL_ERROR "Missing example source: ${_dxl_src}")
  endif()

  add_executable(${_dxl_example} "${_dxl_src}")
  target_link_libraries(${_dxl_example} PRIVATE dynamixel_sdk_c::dynamixel_sdk_c)
endforeach()

