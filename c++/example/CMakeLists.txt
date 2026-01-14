cmake_minimum_required(VERSION 3.10)
project(dynamixel_sdk_cpp_examples)

# DynamixelSDK C++ Examples

if(NOT TARGET dynamixel_cpp)
  message(WARNING "dynamixel_cpp target not found. Examples might fail to build if library is not built.")
endif()

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "*.cpp")

# Group sources by directory
foreach(SOURCE_FILE ${SOURCES})
  if(${SOURCE_FILE} MATCHES "CMakeCompilerId" OR ${SOURCE_FILE} MATCHES "^build/")
    continue()
  endif()

  get_filename_component(PARENT_DIR ${SOURCE_FILE} DIRECTORY)
  
  string(REPLACE "/" "_" DIR_KEY "${PARENT_DIR}")
  string(REPLACE "." "_" DIR_KEY "${DIR_KEY}")
  
  if(DIR_KEY STREQUAL "")
      set(DIR_KEY "root")
  endif()
  
  list(APPEND "SOURCES_${DIR_KEY}" "${SOURCE_FILE}")
  list(APPEND EX_DIRS "${DIR_KEY}")
endforeach()

if(EX_DIRS)
  list(REMOVE_DUPLICATES EX_DIRS)
endif()

# Create targets
foreach(DIR_KEY ${EX_DIRS})
    set(TARGET_SRC ${SOURCES_${DIR_KEY}})
    
    if(DIR_KEY STREQUAL "root")
         foreach(SRC ${TARGET_SRC})
             get_filename_component(FNAME ${SRC} NAME_WE)
             set(TARGET_NAME "${FNAME}_cpp")
             add_executable(${TARGET_NAME} ${SRC})
             
             if(TARGET dynamixel_cpp)
                target_link_libraries(${TARGET_NAME} PRIVATE dynamixel_cpp)
             else()
                target_link_libraries(${TARGET_NAME} PRIVATE dxl_cpp)
             endif()
             
             target_include_directories(${TARGET_NAME} PRIVATE
               ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_sdk
               ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_easy_sdk
             )
         endforeach()
    else()
        set(TARGET_NAME "${DIR_KEY}_cpp")
        
        add_executable(${TARGET_NAME} ${TARGET_SRC})
             
        if(TARGET dynamixel_cpp)
           target_link_libraries(${TARGET_NAME} PRIVATE dynamixel_cpp)
        else()
           target_link_libraries(${TARGET_NAME} PRIVATE dxl_cpp)
        endif()
             
        target_include_directories(${TARGET_NAME} PRIVATE
           ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_sdk
           ${CMAKE_CURRENT_SOURCE_DIR}/../include/dynamixel_easy_sdk
        )
    endif()
endforeach()
