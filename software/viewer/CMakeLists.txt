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

#add_subdirectory(src/renderers)
add_subdirectory(src/renderer_robot_state)
add_subdirectory(src/renderer_robot_plan)
add_subdirectory(src/renderer_affordances)
add_subdirectory(src/renderer_end_effector_goal)
add_subdirectory(src/renderer_manipulation)
add_subdirectory(src/renderer_walking)
add_subdirectory(src/renderer_driving)
add_subdirectory(src/renderer_scrollingplots)
add_subdirectory(src/renderer_status)
add_subdirectory(src/viewer)
