cmake_minimum_required(VERSION 3.10)
project(dynamixel_sdk_c_examples C)

# ==============================================================================
# DynamixelSDK C Examples Build Configuration
# ==============================================================================

# ------------------------------------------------------------------------------
# 1. Dependency Discovery
# ------------------------------------------------------------------------------
if(NOT TARGET dxl_c AND NOT TARGET dynamixel_c)
    message(STATUS "Target 'dxl_c' not defined. Searching system paths...")
    find_library(DXL_C_LIB_SYS NAMES dxl_c dynamixel_c dxl_sbc_c)
    
    if(DXL_C_LIB_SYS)
        add_library(dxl_c UNKNOWN IMPORTED)
        set_property(TARGET dxl_c PROPERTY IMPORTED_LOCATION "${DXL_C_LIB_SYS}")
    else()
        message(FATAL_ERROR "Dynamixel C Library not found! Please build/install the library first.")
    endif()
endif()

# ------------------------------------------------------------------------------
# 2. Helper Function
#    - target_base_name: The desired output filename (e.g., read_write_protocol1)
# ------------------------------------------------------------------------------
function(add_dxl_c_example target_base_name source_file)
    # A. Define Internal Target Name (Append _c automatically)
    set(INTERNAL_TARGET "${target_base_name}_c")

    add_executable(${INTERNAL_TARGET} ${source_file})

    # B. Set the Output Filename (What the user sees)
    #    Force the output binary to match the base name (no _c suffix)
    set_target_properties(${INTERNAL_TARGET} PROPERTIES OUTPUT_NAME "${target_base_name}")

    # C. Link Libraries
    if(TARGET dxl_c)
        target_link_libraries(${INTERNAL_TARGET} PRIVATE dxl_c)
    elseif(TARGET dynamixel_c)
        target_link_libraries(${INTERNAL_TARGET} PRIVATE dynamixel_c)
    else()
        target_link_libraries(${INTERNAL_TARGET} PRIVATE dxl_sbc_c) 
    endif()

    # D. Include Directories
    target_include_directories(${INTERNAL_TARGET} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_sdk
    )
endfunction()

# ==============================================================================
# 3. Explicit Target Definitions
#    Pass the desired output filename as the first argument.
#    The function will automatically append '_c' for the internal CMake target.
# ==============================================================================

# --- Protocol 1.0 Examples ---
add_dxl_c_example(read_write_protocol1      protocol1.0/read_write/read_write.c)
add_dxl_c_example(ping_protocol1            protocol1.0/ping/ping.c)
add_dxl_c_example(multi_port_protocol1      protocol1.0/multi_port/multi_port.c)
add_dxl_c_example(reset_protocol1           protocol1.0/reset/reset.c)
add_dxl_c_example(sync_write_protocol1      protocol1.0/sync_write/sync_write.c)
add_dxl_c_example(bulk_read_protocol1       protocol1.0/bulk_read/bulk_read.c)

# --- Protocol 2.0 Examples ---
add_dxl_c_example(read_write_protocol2       protocol2.0/read_write/read_write.c)
add_dxl_c_example(ping_protocol2             protocol2.0/ping/ping.c)
add_dxl_c_example(multi_port_protocol2       protocol2.0/multi_port/multi_port.c)
add_dxl_c_example(reboot_protocol2           protocol2.0/reboot/reboot.c)
add_dxl_c_example(factory_reset_protocol2    protocol2.0/factory_reset/factory_reset.c)
add_dxl_c_example(indirect_address_protocol2 protocol2.0/indirect_address/indirect_address.c)
add_dxl_c_example(clear_multi_turn_protocol2 protocol2.0/clear_multi_turn/clear_multi_turn.c)

# --- Advanced Protocol 2.0 Features ---
add_dxl_c_example(broadcast_ping_protocol2   protocol2.0/broadcast_ping/broadcast_ping.c)
add_dxl_c_example(sync_read_write_protocol2  protocol2.0/sync_read_write/sync_read_write.c)
add_dxl_c_example(bulk_read_write_protocol2  protocol2.0/bulk_read_write/bulk_read_write.c)

# --- General / Utility Examples ---
add_dxl_c_example(protocol_combined          protocol_combined/protocol_combined.c)
add_dxl_c_example(dxl_monitor                dxl_monitor/dxl_monitor.c)

message(STATUS "Dynamixel SDK C Examples configured (Output names cleaned).")