# iSAM master cmake file
# Michael Kaess, 2010

project(isam)

cmake_minimum_required (VERSION 2.6)

if(CMAKE_BINARY_DIR STREQUAL PROJECT_SOURCE_DIR)
  message(FATAL_ERROR "\nPlease do not build in the source directory, but instead build in a separate directory. The iSAM Makefile provides useful shortcuts for using cmake. Type:\nmake distclean\nto remove any local cmake build files. Then simply type\nmake\nwhich will call cmake in the build/ directory.")
endif()

if(NOT DEFINED CMAKE_BUILD_TYPE OR CMAKE_BUILD_TYPE STREQUAL "")
  set(CMAKE_BUILD_TYPE Release CACHE STRING
    "Choose the type of build, options are: None(CMAKE_CXX_FLAGS or CMAKE_C_FLAGS used) Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()
message(STATUS "Build configuration: ${CMAKE_BUILD_TYPE}")

set (LIBRARY_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/lib" CACHE PATH
  "Target directory for all libraries.")
set (EXECUTABLE_OUTPUT_PATH "${PROJECT_SOURCE_DIR}/bin" CACHE PATH
  "Target directory for all executables.")

# do not edit - use ccmake to change
option (PROFILE "Enable profiling" OFF)
option (USE_LCM "Compile with LCM interface (lcm library needed)" OFF)
option (USE_GUI "Compile with GUI (SDL library needed)" ON)

add_definitions(-Wall)
# note: -g automatic for Debug mode, -O3 -DNDEBUG automatic for Release

if(PROFILE)
  message(STATUS "Profiling active")
  add_definitions(-pg)
endif(PROFILE)

if(USE_LCM)
  add_definitions(-DUSE_LCM)
endif(USE_LCM)
if(USE_GUI)
  add_definitions(-DUSE_GUI)
endif(USE_GUI)

add_definitions(-I${PROJECT_SOURCE_DIR}/include)

add_subdirectory(isamlib)
add_subdirectory(isam)
add_subdirectory(examples)
