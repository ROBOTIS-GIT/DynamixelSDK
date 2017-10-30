################################################################################
# CMake
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_sdk)

################################################################################
# Packages
################################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

################################################################################
# Declare ROS messages, services and actions
################################################################################

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Catkin specific configuration
################################################################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamixel_sdk
)

################################################################################
# Build
################################################################################
include_directories(
  include/dynamixel_sdk
  ${catkin_INCLUDE_DIRS}
)

add_library(dynamixel_sdk
  src/dynamixel_sdk/packet_handler.cpp
  src/dynamixel_sdk/protocol1_packet_handler.cpp
  src/dynamixel_sdk/protocol2_packet_handler.cpp
  src/dynamixel_sdk/group_sync_read.cpp
  src/dynamixel_sdk/group_sync_write.cpp
  src/dynamixel_sdk/group_bulk_read.cpp
  src/dynamixel_sdk/group_bulk_write.cpp
  src/dynamixel_sdk/port_handler.cpp
)

#if(APPLE)
#  target_sources(dynamixel_sdk PRIVATE src/dynamixel_sdk/port_handler_mac.cpp)
#else()
#  target_sources(dynamixel_sdk PRIVATE src/dynamixel_sdk/port_handler_linux.cpp)
#endif()

if(APPLE)
  set_property(TARGET dynamixel_sdk APPEND PROPERTY SOURCES src/dynamixel_sdk/port_handler_mac.cpp)
else()
  set_property(TARGET dynamixel_sdk APPEND PROPERTY SOURCES src/dynamixel_sdk/port_handler_linux.cpp)
endif()

#if(APPLE)
#  target_sources(dynamixel_sdk PRIVATE src/dynamixel_sdk/port_handler_mac.cpp)
#else()
#set_property(dynamixel_sdk PRIVATE
#  src/dynamixel_sdk/port_handler_linux.cpp
#)
#endif()

add_dependencies(dynamixel_sdk ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(dynamixel_sdk ${catkin_LIBRARIES})

################################################################################
# Install
################################################################################
install(TARGETS dynamixel_sdk
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

################################################################################
# Test
################################################################################
