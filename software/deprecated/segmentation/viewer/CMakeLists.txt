cmake_minimum_required(VERSION 2.6.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME surrogate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build libfreenect
#set(LIB_SUFFIX "" CACHE STRING "Suffix for libfreenect installation directory")
#add_subdirectory(libfreenect)

include_directories(${LCMTYPES_INCLUDE_DIRS})
#tuna
add_subdirectory(src/surrogate_gui)
