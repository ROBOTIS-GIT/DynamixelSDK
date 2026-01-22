cmake_minimum_required(VERSION 3.10)
project(dynamixel_sdk_cpp_examples)

# ==============================================================================
# DynamixelSDK C++ Examples Build Configuration
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Dependency Discovery
# ------------------------------------------------------------------------------
if(NOT TARGET dxl_cpp AND NOT TARGET dynamixel_cpp)
    message(STATUS "Target 'dxl_cpp' not defined. Searching system paths...")
    find_library(DXL_LIB_SYS NAMES dxl_cpp dynamixel_cpp dxl_sbc_cpp)
    
    if(DXL_LIB_SYS)
        add_library(dxl_cpp UNKNOWN IMPORTED)
        set_property(TARGET dxl_cpp PROPERTY IMPORTED_LOCATION "${DXL_LIB_SYS}")
    else()
        message(FATAL_ERROR "Dynamixel C++ Library not found! Please build/install the library first.")
    endif()
endif()

# ------------------------------------------------------------------------------
# 2. Helper Function with OUTPUT_NAME Property
#    - target_base_name: The desired output filename (e.g., read_write_protocol1)
# ------------------------------------------------------------------------------
function(add_dxl_example target_base_name source_file)
    # A. Define Internal Target Name (Append _cpp to ensure uniqueness)
    set(INTERNAL_TARGET "${target_base_name}_cpp")
    
    add_executable(${INTERNAL_TARGET} ${source_file})

    # B. Set the Output Filename (What the user sees)
    #    Force the output binary to be named 'read_write_protocol1', not 'read_write_protocol1_cpp'
    set_target_properties(${INTERNAL_TARGET} PROPERTIES OUTPUT_NAME "${target_base_name}")

    # C. Link Libraries
    if(TARGET dxl_cpp)
        target_link_libraries(${INTERNAL_TARGET} PRIVATE dxl_cpp)
    elseif(TARGET dynamixel_cpp)
        target_link_libraries(${INTERNAL_TARGET} PRIVATE dynamixel_cpp)
    else()
        target_link_libraries(${INTERNAL_TARGET} PRIVATE dxl_sbc_cpp) 
    endif()

    # D. Include Directories
    target_include_directories(${INTERNAL_TARGET} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_sdk
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_easy_sdk
    )
endfunction()

# ==============================================================================
# 3. Explicit Target Definitions
#    Pass the desired output filename as the first argument.
#    The function will automatically append '_cpp' for the internal CMake target.
# ==============================================================================

# --- Protocol 1.0 Examples ---
add_dxl_example(read_write_protocol1      protocol1.0/read_write/read_write.cpp)
add_dxl_example(ping_protocol1            protocol1.0/ping/ping.cpp)
add_dxl_example(multi_port_protocol1      protocol1.0/multi_port/multi_port.cpp)
add_dxl_example(reset_protocol1           protocol1.0/reset/reset.cpp)
add_dxl_example(sync_write_protocol1      protocol1.0/sync_write/sync_write.cpp)
add_dxl_example(bulk_read_protocol1       protocol1.0/bulk_read/bulk_read.cpp)

# --- Protocol 2.0 Examples ---
add_dxl_example(read_write_protocol2      protocol2.0/read_write/read_write.cpp)
add_dxl_example(ping_protocol2            protocol2.0/ping/ping.cpp)
add_dxl_example(multi_port_protocol2      protocol2.0/multi_port/multi_port.cpp)
add_dxl_example(reboot_protocol2          protocol2.0/reboot/reboot.cpp)
add_dxl_example(factory_reset_protocol2   protocol2.0/factory_reset/factory_reset.cpp)
add_dxl_example(indirect_address_protocol2 protocol2.0/indirect_address/indirect_address.cpp)
add_dxl_example(clear_multi_turn_protocol2 protocol2.0/clear_multi_turn/clear_multi_turn.cpp)

# --- Advanced Protocol 2.0 Features ---
add_dxl_example(broadcast_ping_protocol2  protocol2.0/broadcast_ping/broadcast_ping.cpp)
add_dxl_example(sync_read_write_protocol2 protocol2.0/sync_read_write/sync_read_write.cpp)
add_dxl_example(fast_sync_read_protocol2  protocol2.0/fast_sync_read/fast_sync_read.cpp)
add_dxl_example(bulk_read_write_protocol2 protocol2.0/bulk_read_write/bulk_read_write.cpp)
add_dxl_example(fast_bulk_read_protocol2  protocol2.0/fast_bulk_read/fast_bulk_read.cpp)

# --- General / Utility Examples ---
add_dxl_example(protocol_combined         protocol_combined/protocol_combined.cpp)
add_dxl_example(dxl_monitor               dxl_monitor/dxl_monitor.cpp)

message(STATUS "Dynamixel SDK C++ Examples configured (Output names cleaned).")