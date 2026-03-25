# Build protocol2.0 C examples using installed dynamixel_sdk_c package.
cmake_minimum_required(VERSION 3.10)

project(dynamixel_sdk_c_examples_protocol2 LANGUAGES C)

find_package(dynamixel_sdk_c CONFIG REQUIRED)

set(_dxl_sdk_target dynamixel_sdk_c::dynamixel_sdk_c)
set(_dxl_extra_includes "")
set(_dxl_protocol2_examples
  broadcast_ping
  ping
  read_write
  sync_read_write
  reboot
  multi_port
  clear_multi_turn
  factory_reset
  indirect_address
  bulk_read_write
)

foreach(_dxl_example IN LISTS _dxl_protocol2_examples)
  set(_dxl_src "${CMAKE_CURRENT_SOURCE_DIR}/${_dxl_example}/${_dxl_example}.c")
  if(NOT EXISTS "${_dxl_src}")
    message(FATAL_ERROR "Missing example source: ${_dxl_src}")
  endif()

  add_executable(${_dxl_example} "${_dxl_src}")
  if(NOT _dxl_extra_includes STREQUAL "")
    target_include_directories(${_dxl_example} PRIVATE ${_dxl_extra_includes})
  endif()
  target_link_libraries(${_dxl_example} PRIVATE ${_dxl_sdk_target})
endforeach()
