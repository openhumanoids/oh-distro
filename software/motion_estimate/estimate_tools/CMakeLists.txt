cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME estimate_tools)
include(cmake/pods.cmake)

SET(CMAKE_BUILD_TYPE Debug)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()



add_subdirectory(src/filter_tools)
add_subdirectory(src/estimate_tools)
add_subdirectory(src/kalman_filter_tools)
add_subdirectory(src/backlash_filter_tools)

add_subdirectory(src/test)

