################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(dynamixel_sdk)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-but-set-variable -Wno-unused-parameter -Wno-unused-variable)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

################################################################################
# Build
################################################################################
include_directories(
  include/${PROJECT_NAME}
)

set(DYNAMIXEL_SDK_SOURCES
  src/dynamixel_sdk/packet_handler.cpp
  src/dynamixel_sdk/protocol1_packet_handler.cpp
  src/dynamixel_sdk/protocol2_packet_handler.cpp
  src/dynamixel_sdk/group_sync_read.cpp
  src/dynamixel_sdk/group_sync_write.cpp
  src/dynamixel_sdk/group_bulk_read.cpp
  src/dynamixel_sdk/group_bulk_write.cpp
  src/dynamixel_sdk/group_fast_bulk_read.cpp
  src/dynamixel_sdk/group_fast_sync_read.cpp
  src/dynamixel_sdk/group_handler.cpp
  src/dynamixel_sdk/port_handler.cpp
)

if(APPLE)
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/dynamixel_sdk/port_handler_mac.cpp
  )
elseif(WIN32)
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/dynamixel_sdk/port_handler_windows.cpp
  )
else()
  add_library(dynamixel_sdk SHARED
    ${DYNAMIXEL_SDK_SOURCES}
    src/dynamixel_sdk/port_handler_linux.cpp
  )
endif()

################################################################################
# Install
################################################################################
install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

################################################################################
# Install Python Package
################################################################################
ament_python_install_package(${PROJECT_NAME} PACKAGE_DIR "src/dynamixel_sdk")

################################################################################
# Macro for ament package
################################################################################
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_package()
