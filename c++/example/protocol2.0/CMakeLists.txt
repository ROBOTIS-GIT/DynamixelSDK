cmake_minimum_required(VERSION 3.10)

# Standalone build for C++ Protocol 2.0 examples (uses installed dynamixel_sdk).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(dynamixel_sdk_cpp_examples_protocol2 LANGUAGES CXX)
endif()

find_package(dynamixel_sdk CONFIG REQUIRED)

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
  fast_sync_read
  fast_bulk_read
  bulk_read_write
)

foreach(_dxl_example IN LISTS _dxl_protocol2_examples)
  set(_dxl_src "${CMAKE_CURRENT_SOURCE_DIR}/${_dxl_example}/${_dxl_example}.cpp")
  if(NOT EXISTS "${_dxl_src}")
    message(FATAL_ERROR "Missing example source: ${_dxl_src}")
  endif()

  set(_dxl_target "${_dxl_example}")
  add_executable(${_dxl_target} "${_dxl_src}")
  target_link_libraries(${_dxl_target} PRIVATE dynamixel_sdk::dynamixel_sdk)
endforeach()

