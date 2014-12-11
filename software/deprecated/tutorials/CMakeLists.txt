cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mfallon_sandbox)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/lcmgl_example)
add_subdirectory(src/image-simple)

# basic ncurses implementation:
add_subdirectory(src/ncurses_example)


add_subdirectory(src/perception_example)
add_subdirectory(src/modified_stereo_odometry)
