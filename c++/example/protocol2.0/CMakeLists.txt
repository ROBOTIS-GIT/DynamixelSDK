cmake_minimum_required(VERSION 3.10)

# Standalone build for C++ Protocol 2.0 examples (uses installed dynamixel_sdk).
if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
  project(dynamixel_sdk_cpp_examples_protocol2 LANGUAGES CXX)
endif()

find_package(dynamixel_sdk CONFIG REQUIRED)

set(_dxl_sdk_target dynamixel_sdk::dynamixel_sdk)

# The examples use `#include "dynamixel_sdk.h"`, so we need the include dir
# that contains that header. Typically it's `${prefix}/include/dynamixel_sdk`.
set(_dxl_extra_includes "")
get_target_property(_dxl_iface_includes ${_dxl_sdk_target} INTERFACE_INCLUDE_DIRECTORIES)
foreach(_dxl_inc IN LISTS _dxl_iface_includes)
  list(APPEND _dxl_extra_includes "${_dxl_inc}/dynamixel_sdk")
endforeach()

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

  set(_dxl_target "p2_${_dxl_example}")
  add_executable(${_dxl_target} "${_dxl_src}")
  target_include_directories(${_dxl_target} PRIVATE
    ${_dxl_extra_includes}
  )
  target_link_libraries(${_dxl_target} PRIVATE ${_dxl_sdk_target})
endforeach()

