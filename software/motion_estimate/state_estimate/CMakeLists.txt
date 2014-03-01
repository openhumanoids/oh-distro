cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME state_estimate)
include(cmake/pods.cmake)

SET(CMAKE_BUILD_TYPE Debug)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


add_subdirectory(src/basic_legodo)
add_subdirectory(src/state_estimate)

