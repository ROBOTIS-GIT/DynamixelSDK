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

# --------------------------------------------------------
# 2. Build C Library (libdxl_c)
# --------------------------------------------------------
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

target_include_directories(dynamixel_c PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/c/include>
  $<INSTALL_INTERFACE:include>
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

# --------------------------------------------------------
# 3. Build C++ Library (libdxl_cpp)
# --------------------------------------------------------
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

# --------------------------------------------------------
# 4. Installation & Export
# --------------------------------------------------------
include(GNUInstallDirs)

install(DIRECTORY c/include/dynamixel_sdk/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dynamixel_sdk)
install(DIRECTORY c++/include/dynamixel_sdk/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/dynamixel_sdk)

# A. Installation for ROS 2 Environment
if(IS_ROS_BUILD)
  # 1) Libraries
  install(TARGETS dynamixel_c dynamixel_cpp
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
  )

  # 2) Python Package
  find_package(ament_cmake_python REQUIRED)
  ament_python_install_package(${PROJECT_NAME}
    PACKAGE_DIR python/src/dynamixel_sdk
  )

  # 3) Export
  ament_export_include_directories(c/include c++/include)
  ament_export_libraries(dynamixel_c dynamixel_cpp)
  ament_export_dependencies(ament_cmake_python python3-serial)
  
  ament_package()

# B. Installation for Standalone System
else()
  install(TARGETS dynamixel_c dynamixel_cpp
    EXPORT dynamixel_sdk_targets
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()
