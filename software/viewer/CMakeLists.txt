#main
cmake_minimum_required(VERSION 2.6.0)

file(GLOB hpp_files include/visualization/*.hpp)
#add_definitions(-Wall -msse2 -msse3)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME viewer)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

add_subdirectory(src/renderer_driving)
add_subdirectory(src/renderers)
add_subdirectory(src/viewer)
