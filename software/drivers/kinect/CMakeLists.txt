cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME kinect)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build libfreenect
set(LIB_SUFFIX "" CACHE STRING "Suffix for libfreenect installation directory")
add_subdirectory(libfreenect)
include_directories(/usr/include/ni)

include_directories(${LCMTYPES_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/libfreenect/include)


# build LCM kinect utility
add_subdirectory(src/kinect-lcm)

# kinect utility library
add_subdirectory(src/kinect-utils)

# simple GLUT visualization tool
add_subdirectory(src/kinect-glview)

# bot2-vis renderer and viewer
add_subdirectory(src/kinect-renderer)

add_subdirectory(src/kinect-openni-lcm)
