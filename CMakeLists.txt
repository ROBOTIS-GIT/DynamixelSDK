cmake_minimum_required(VERSION 3.10)

# Define project and supported languages (C, C++)
project(dynamixel_sdk VERSION 4.0.4 LANGUAGES C CXX)

# --------------------------------------------------------
# 1. Build Environment Configuration
# --------------------------------------------------------
# C++ Standard Settings (Recommend C++17)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# C Standard Settings (C99)
set(CMAKE_C_STANDARD 99)

# Position Independent Code (fPIC) Settings
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# [Legacy Support] Determine Library Name Aliases based on Architecture
set(LEGACY_CXX_NAME "")
set(LEGACY_C_NAME "")

if(APPLE)
  set(LEGACY_C_NAME   dxl_mac_c)
  set(LEGACY_CXX_NAME dxl_mac_cpp)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64)$")
  set(LEGACY_CXX_NAME "dxl_x64_cpp")
  set(LEGACY_C_NAME   "dxl_x64_c")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(i.86|x86)$")
  set(LEGACY_CXX_NAME "dxl_x86_cpp")
  set(LEGACY_C_NAME   "dxl_x86_c")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|arm64|aarch64)$")
  set(LEGACY_CXX_NAME "dxl_sbc_cpp")
  set(LEGACY_C_NAME   "dxl_sbc_c")
endif()

# Detect ROS 2 Environment
find_package(ament_cmake QUIET)

# Set the default value based on whether ament_cmake was found, but allow user override
option(IS_ROS_BUILD "Build with ROS 2 support" ${ament_cmake_FOUND})

if(IS_ROS_BUILD)
  message(STATUS "Build Environment: ROS 2 Support ON")
  # If explicitly enabled, ensure ament_cmake is present
  find_package(ament_cmake REQUIRED)
else()
  message(STATUS "Build Environment: Standalone (System Install)")
endif()

# [Improvement] Build Options for Selecting Libraries
option(BUILD_C_LIB "Build C Library" ON)
option(BUILD_CXX_LIB "Build C++ Library" ON)

if(NOT BUILD_C_LIB AND NOT BUILD_CXX_LIB)
  message(FATAL_ERROR "Both BUILD_C_LIB and BUILD_CXX_LIB are set to OFF. At least one must be enabled.")
endif()

# --------------------------------------------------------
# 2. Build C Library (libdxl_c)
# --------------------------------------------------------
if(BUILD_C_LIB)
  # Common source files (platform-independent)
  set(DXL_C_COMMON_SOURCES
    c/src/dynamixel_sdk/group_bulk_read.c
    c/src/dynamixel_sdk/group_bulk_write.c
    c/src/dynamixel_sdk/group_sync_read.c
    c/src/dynamixel_sdk/group_sync_write.c
    c/src/dynamixel_sdk/packet_handler.c
    c/src/dynamixel_sdk/protocol1_packet_handler.c
    c/src/dynamixel_sdk/protocol2_packet_handler.c
    c/src/dynamixel_sdk/port_handler.c
  )

  # Platform-specific port handler
  if(WIN32)
    set(DXL_C_PLATFORM_SOURCES c/src/dynamixel_sdk/port_handler_windows.c)
  elseif(APPLE)
    set(DXL_C_PLATFORM_SOURCES c/src/dynamixel_sdk/port_handler_mac.c)
  else() # Linux / UNIX
    set(DXL_C_PLATFORM_SOURCES c/src/dynamixel_sdk/port_handler_linux.c)
  endif()

  add_library(dynamixel_c SHARED
    ${DXL_C_COMMON_SOURCES}
    ${DXL_C_PLATFORM_SOURCES}
  )

  # C users must include the special directory to avoid conflict with C++ headers
  # Usage: #include "dynamixel_sdk/dynamixel_sdk.h" (Finds file in include/dynamixel_sdk_c/dynamixel_sdk/)
  target_include_directories(dynamixel_c PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/c/include>
    $<INSTALL_INTERFACE:include/dynamixel_sdk_c>
  )

  target_include_directories(dynamixel_c PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/c/include/dynamixel_sdk
  )

  set_target_properties(dynamixel_c PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 2
    OUTPUT_NAME "dxl_c"
  )

  if(UNIX AND NOT APPLE)
    target_link_libraries(dynamixel_c PRIVATE rt)
  endif()
endif()

# --------------------------------------------------------
# 3. Build C++ Library (libdxl_cpp)
# --------------------------------------------------------
if(BUILD_CXX_LIB)
  # Common SDK sources (platform-independent)
  set(DXL_CPP_COMMON_SOURCES
    c++/src/dynamixel_sdk/group_bulk_read.cpp
    c++/src/dynamixel_sdk/group_bulk_write.cpp
    c++/src/dynamixel_sdk/group_fast_bulk_read.cpp
    c++/src/dynamixel_sdk/group_fast_sync_read.cpp
    c++/src/dynamixel_sdk/group_handler.cpp
    c++/src/dynamixel_sdk/group_sync_read.cpp
    c++/src/dynamixel_sdk/group_sync_write.cpp
    c++/src/dynamixel_sdk/packet_handler.cpp
    c++/src/dynamixel_sdk/protocol1_packet_handler.cpp
    c++/src/dynamixel_sdk/protocol2_packet_handler.cpp
    c++/src/dynamixel_sdk/port_handler.cpp
  )

  # Platform-specific port handler
  if(WIN32)
    set(DXL_CPP_PLATFORM_SOURCES c++/src/dynamixel_sdk/port_handler_windows.cpp)
  elseif(APPLE)
    set(DXL_CPP_PLATFORM_SOURCES c++/src/dynamixel_sdk/port_handler_mac.cpp)
  else() # Linux / UNIX
    set(DXL_CPP_PLATFORM_SOURCES c++/src/dynamixel_sdk/port_handler_linux.cpp)
  endif()

  # Easy SDK sources
  set(DXL_CPP_EASY_SOURCES
    c++/src/dynamixel_easy_sdk/connector.cpp
    c++/src/dynamixel_easy_sdk/control_table.cpp
    c++/src/dynamixel_easy_sdk/dynamixel_error.cpp
    c++/src/dynamixel_easy_sdk/group_executor.cpp
    c++/src/dynamixel_easy_sdk/motor.cpp
  )

  add_library(dynamixel_cpp SHARED
    ${DXL_CPP_COMMON_SOURCES}
    ${DXL_CPP_PLATFORM_SOURCES}
    ${DXL_CPP_EASY_SOURCES}
  )

  target_include_directories(dynamixel_cpp PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/c++/include>
    $<INSTALL_INTERFACE:include>
  )
  target_include_directories(dynamixel_cpp PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/c++/include/dynamixel_sdk
    ${CMAKE_CURRENT_SOURCE_DIR}/c++/include/dynamixel_easy_sdk
  )

  set_target_properties(dynamixel_cpp PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 2
    OUTPUT_NAME "dxl_cpp"
  )

  if(IS_ROS_BUILD)
    find_package(ament_index_cpp REQUIRED)
    target_link_libraries(dynamixel_cpp PRIVATE ament_index_cpp::ament_index_cpp)
    target_compile_definitions(dynamixel_cpp PRIVATE ROS_BUILD)
  endif()

  if(UNIX AND NOT APPLE)
    target_link_libraries(dynamixel_cpp PRIVATE pthread rt)
  endif()
endif()

# --------------------------------------------------------
# Umbrella target for ROS consumers
# --------------------------------------------------------
if(IS_ROS_BUILD AND BUILD_CXX_LIB)
  add_library(dynamixel_sdk INTERFACE)

  target_include_directories(dynamixel_sdk INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/c++/include>
    $<INSTALL_INTERFACE:include>
  )

  target_link_libraries(dynamixel_sdk INTERFACE
    dynamixel_cpp
  )

  if(BUILD_C_LIB)
    target_link_libraries(dynamixel_sdk INTERFACE
      dynamixel_c
    )
  endif()
endif()

# --------------------------------------------------------
# 4. Installation & Export
# --------------------------------------------------------
include(GNUInstallDirs)

# [Clean Install Strategy]
# C++ headers go to standard include/dynamixel_sdk (Default)
# C headers go to include/dynamixel_sdk_c/dynamixel_sdk (Isolated)

if(BUILD_C_LIB)
  install(DIRECTORY c/include/dynamixel_sdk/ DESTINATION include/dynamixel_sdk_c/dynamixel_sdk)
endif()

if(BUILD_CXX_LIB)
  install(DIRECTORY c++/include/dynamixel_sdk/ DESTINATION include/dynamixel_sdk)
  install(DIRECTORY c++/include/dynamixel_easy_sdk/ DESTINATION include/dynamixel_easy_sdk)
endif()

  # --------------------------------------------------------
  # Feature: Uninstall and Reinstall Targets
  # Usage: sudo make uninstall / sudo make reinstall
  # --------------------------------------------------------
  
  # [Modified] Use configure_file instead of writing the script inline string
  # This generates the uninstall script from the template file.
  configure_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
    IMMEDIATE @ONLY
  )

  # Add custom targets
  if(NOT TARGET uninstall)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
      COMMENT "Uninstalling DynamixelSDK components..."
    )
  endif()

  if(NOT TARGET reinstall)
    add_custom_target(reinstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
      COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR} --target install
      COMMENT "Reinstalling DynamixelSDK components..."
    )
  endif()

# A. Installation for ROS 2 Environment
if(IS_ROS_BUILD)
  set(EXPORT_TARGETS)
  if(BUILD_C_LIB)
    list(APPEND EXPORT_TARGETS dynamixel_c)
  endif()
  if(BUILD_CXX_LIB)
    list(APPEND EXPORT_TARGETS dynamixel_cpp)
  endif()
  if(BUILD_CXX_LIB)
    list(APPEND EXPORT_TARGETS dynamixel_sdk)
  endif()

  install(TARGETS ${EXPORT_TARGETS}
    EXPORT dynamixel_sdk_targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )
  
  install(EXPORT dynamixel_sdk_targets
    FILE dynamixel_sdk_targets.cmake
    NAMESPACE dynamixel_sdk::
    DESTINATION share/${PROJECT_NAME}/cmake
  )

  # Python installation is generally tied to the SDK utility, so we install it if ROS build is on.
  # (Python bindings depend on C libraries usually via ctypes/serial, but here it's pure python)
  find_package(ament_cmake_python REQUIRED)
  ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR python/src/dynamixel_sdk
  )
  ament_python_install_package(dynamixel_easy_sdk
    PACKAGE_DIR python/src/dynamixel_easy_sdk
  )

  # Export Include Directories
  if(BUILD_CXX_LIB)
    ament_export_include_directories(include)
  endif()
  
  # Only export C include directory if C lib is built.
  if(BUILD_C_LIB)
    ament_export_include_directories(include/dynamixel_sdk_c)
  endif()
  
  ament_export_targets(dynamixel_sdk_targets HAS_LIBRARY_TARGET)
  
  # For legacy CMake support variables
  set(LEGACY_LIBRARIES "")
  if(BUILD_C_LIB)
    list(APPEND LEGACY_LIBRARIES dxl_c)
  endif()
  if(BUILD_CXX_LIB)
    list(APPEND LEGACY_LIBRARIES dxl_cpp)
  endif()
  
  ament_export_libraries(${LEGACY_LIBRARIES})
  # ament_export_dependencies() # No Runtime CMake dependencies to export
  
  ament_package()

# B. Installation for Standalone System
else()
  # Same logic for standalone
  set(EXPORT_TARGETS)
  if(BUILD_C_LIB)
    list(APPEND EXPORT_TARGETS dynamixel_c)
  endif()
  if(BUILD_CXX_LIB)
    list(APPEND EXPORT_TARGETS dynamixel_cpp)
  endif()

  install(TARGETS ${EXPORT_TARGETS}
    EXPORT dynamixel_sdk_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )

  # Install Control Table Models (Essential for Easy SDK)
  install(DIRECTORY control_table DESTINATION share/dynamixel_sdk)

  # [Legacy Support] Create Symlinks for Backward Compatibility
  if(UNIX)
    if(LEGACY_CXX_NAME AND BUILD_CXX_LIB)
      install(CODE "
        message(STATUS \"Creating legacy symlink: lib${LEGACY_CXX_NAME}.so -> libdxl_cpp.so\")
        execute_process(
          COMMAND \${CMAKE_COMMAND} -E create_symlink
                  libdxl_cpp.so
                  \${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/lib${LEGACY_CXX_NAME}.so
          WORKING_DIRECTORY \"\$ENV{DESTDIR}\"
        )
      ")
    endif()

    if(LEGACY_C_NAME AND BUILD_C_LIB)
      install(CODE "
        message(STATUS \"Creating legacy symlink: lib${LEGACY_C_NAME}.so -> libdxl_c.so\")
        execute_process(
          COMMAND \${CMAKE_COMMAND} -E create_symlink
                  libdxl_c.so
                  \${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/lib${LEGACY_C_NAME}.so
          WORKING_DIRECTORY \"\$ENV{DESTDIR}\"
        )
      ")
    endif()
  endif()

endif()
