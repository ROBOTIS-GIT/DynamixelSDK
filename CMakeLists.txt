cmake_minimum_required(VERSION 3.10)

# Define project and supported languages (C, C++)
project(dynamixel_sdk VERSION 4.0.3 LANGUAGES C CXX)

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

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(LEGACY_CXX_NAME "dxl_x64_cpp")
  set(LEGACY_C_NAME   "dxl_x64_c")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(i.86|x86)$")
  set(LEGACY_CXX_NAME "dxl_x86_cpp")
  set(LEGACY_C_NAME   "dxl_x86_c")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(arm|aarch64)$")
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
  file(GLOB_RECURSE C_SOURCES "c/src/dynamixel_sdk/*.c")

  # Filter source files based on the platform to avoid compiling unsupported OS code
  if(WIN32)
    list(FILTER C_SOURCES EXCLUDE REGEX "port_handler_(linux|mac)\\.c$")
  elseif(APPLE)
    list(FILTER C_SOURCES EXCLUDE REGEX "port_handler_(linux|windows)\\.c$")
  else() # Linux / UNIX
    list(FILTER C_SOURCES EXCLUDE REGEX "port_handler_(windows|mac)\\.c$")
  endif()

  add_library(dynamixel_c SHARED ${C_SOURCES})

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
  file(GLOB_RECURSE CXX_SOURCES "c++/src/dynamixel_sdk/*.cpp")
  file(GLOB CXX_EASY_SOURCES "c++/src/dynamixel_easy_sdk/*.cpp")

  # Filter source files based on the platform
  if(WIN32)
    list(FILTER CXX_SOURCES EXCLUDE REGEX "port_handler_(linux|mac)\\.cpp$")
  elseif(APPLE)
    list(FILTER CXX_SOURCES EXCLUDE REGEX "port_handler_(linux|windows)\\.cpp$")
  else() # Linux / UNIX
    list(FILTER CXX_SOURCES EXCLUDE REGEX "port_handler_(windows|mac)\\.cpp$")
  endif()

  add_library(dynamixel_cpp SHARED ${CXX_SOURCES} ${CXX_EASY_SOURCES})

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

  # [Legacy Support] Create Symlinks for Backward Compatibility
  if(UNIX AND NOT APPLE)
      install(CODE "
        if(\"${LEGACY_CXX_NAME}\" AND \"${BUILD_CXX_LIB}\")
          message(STATUS \"Creating legacy symlink: lib${LEGACY_CXX_NAME}.so -> libdxl_cpp.so\")
          execute_process(COMMAND ln -sf libdxl_cpp.so \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/lib${LEGACY_CXX_NAME}.so\")
        endif()
        if(\"${LEGACY_C_NAME}\" AND \"${BUILD_C_LIB}\")
          message(STATUS \"Creating legacy symlink: lib${LEGACY_C_NAME}.so -> libdxl_c.so\")
          execute_process(COMMAND ln -sf libdxl_c.so \"\$ENV{DESTDIR}\${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/lib${LEGACY_C_NAME}.so\")
        endif()
      ")
  endif()

  # --------------------------------------------------------
  # Feature: Uninstall and Reinstall Targets
  # Usage: sudo make uninstall / sudo make reinstall
  # --------------------------------------------------------
  
  # Generate uninstall script dynamically in the BUILD directory (No extra source files needed)
  set(UNINSTALL_SCRIPT "
if(NOT EXISTS \"\${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\")
  message(FATAL_ERROR \"Cannot find install manifest: \\\"\${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\\\"\")
endif()

file(READ \"\${CMAKE_CURRENT_BINARY_DIR}/install_manifest.txt\" files)
string(REGEX REPLACE \"\\n\" \";\" files \"\${files}\")

foreach(file \${files})
  message(STATUS \"Uninstalling \$ENV{DESTDIR}\${file}\")
  if(IS_SYMLINK \"\$ENV{DESTDIR}\${file}\" OR EXISTS \"\$ENV{DESTDIR}\${file}\")
    execute_process(
      COMMAND \${CMAKE_COMMAND} -E remove \"\$ENV{DESTDIR}\${file}\"
      OUTPUT_VARIABLE rm_out
      RESULT_VARIABLE rm_retval
    )
    if(NOT \${rm_retval} EQUAL 0)
      message(FATAL_ERROR \"Problem when removing \$ENV{DESTDIR}\${file}\")
    endif()
  else()
    message(STATUS \"File \$ENV{DESTDIR}\${file} does not exist.\")
  endif()
endforeach()
")

  # Write the script content to the build directory
  file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake" "${UNINSTALL_SCRIPT}")

  # Add custom targets
  if(NOT TARGET uninstall)
    add_custom_target(uninstall
      COMMAND ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
      COMMENT "Uninstalling DynamixelSDK components..."
    )
  endif()

  if(NOT TARGET reinstall)
    add_custom_target(reinstall
      COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR} --target uninstall
      COMMAND ${CMAKE_COMMAND} --build ${CMAKE_CURRENT_BINARY_DIR} --target install
      COMMENT "Reinstalling DynamixelSDK components..."
    )
  endif()

endif()
