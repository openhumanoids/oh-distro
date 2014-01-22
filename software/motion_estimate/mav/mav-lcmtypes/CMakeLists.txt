cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME mav-lcmtypes)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src)
