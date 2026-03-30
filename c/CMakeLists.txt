cmake_minimum_required(VERSION 3.10)

# Standalone C SDK: configure/build from the c/ directory (paths below are relative to c/).
project(dynamixel_sdk_c VERSION 4.0.4 LANGUAGES C)

# --------------------------------------------------------
# 1. Build Environment Configuration
# --------------------------------------------------------
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(DYNAMIXEL_SDK_RUN_LDCONFIG
  "Run ldconfig after install (usually requires root). Recommended OFF."
  OFF
)

# --------------------------------------------------------
# 2. Build C Library (libdxl_c)
# --------------------------------------------------------

set(_dxl_common_sources
  src/dynamixel_sdk/group_bulk_read.c
  src/dynamixel_sdk/group_bulk_write.c
  src/dynamixel_sdk/group_sync_read.c
  src/dynamixel_sdk/group_sync_write.c
  src/dynamixel_sdk/packet_handler.c
  src/dynamixel_sdk/protocol1_packet_handler.c
  src/dynamixel_sdk/protocol2_packet_handler.c
  src/dynamixel_sdk/port_handler.c
)

if(WIN32)
  set(_dxl_platform_sources
    src/dynamixel_sdk/port_handler_windows.c
  )
elseif(APPLE)
  set(_dxl_platform_sources
    src/dynamixel_sdk/port_handler_mac.c
  )
else()
  set(_dxl_platform_sources
    src/dynamixel_sdk/port_handler_linux.c
  )
endif()

set(_dxl_sources
  ${_dxl_common_sources}
  ${_dxl_platform_sources}
)

add_library(dynamixel_sdk_c SHARED ${_dxl_sources})

target_include_directories(dynamixel_sdk_c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  $<INSTALL_INTERFACE:include/dynamixel_sdk_c>
)
# For using lib without dynamixel_sdk_c prefix in Private
target_include_directories(dynamixel_sdk_c PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamixel_sdk
)

set_target_properties(dynamixel_sdk_c PROPERTIES
  VERSION ${PROJECT_VERSION}
  SOVERSION 2
  OUTPUT_NAME "dxl_c"
)

if(UNIX AND NOT APPLE)
  target_link_libraries(dynamixel_sdk_c PRIVATE rt)
endif()

# --------------------------------------------------------
# 3. Installation
# --------------------------------------------------------
include(GNUInstallDirs)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamixel_sdk/
  DESTINATION include/dynamixel_sdk_c
)

set(_dxl_c_cmake_package_dir "${CMAKE_INSTALL_LIBDIR}/cmake/dynamixel_sdk_c")

install(
  TARGETS dynamixel_sdk_c
  EXPORT dynamixel_sdk_cTargets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION include
)

install(
  EXPORT dynamixel_sdk_cTargets
  FILE dynamixel_sdk_cTargets.cmake
  NAMESPACE dynamixel_sdk_c::
  DESTINATION ${_dxl_c_cmake_package_dir}
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/dynamixel_sdk_cConfig.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk_cConfig.cmake"
  INSTALL_DESTINATION ${_dxl_c_cmake_package_dir}
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk_cConfigVersion.cmake"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk_cConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/dynamixel_sdk_cConfigVersion.cmake"
  DESTINATION ${_dxl_c_cmake_package_dir}
)

if(DYNAMIXEL_SDK_RUN_LDCONFIG AND UNIX AND NOT APPLE)
  install(CODE "execute_process(COMMAND ldconfig)")
endif()

# --------------------------------------------------------
# 4. Uninstall / Reinstall Targets
# --------------------------------------------------------
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY
)

if(NOT TARGET uninstall)
  add_custom_target(uninstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    COMMENT "Uninstalling DynamixelSDK (C)..."
  )
endif()

if(NOT TARGET reinstall)
  add_custom_target(reinstall
    COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
    COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR} --target install
    COMMENT "Reinstalling DynamixelSDK (C)..."
  )
endif()

