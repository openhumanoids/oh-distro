cmake_minimum_required(VERSION 2.8.5)
project(DRCExternals)

# set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${DRCExternals_BINARY_DIR})

include(cmake/externals.cmake)
include(cmake/dd-externals.cmake)
