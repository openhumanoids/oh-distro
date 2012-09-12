cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME pointcloud-tools)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()
lcmtypes_build(C_AGGREGATE_HEADER pointcloud_utils.h)

add_subdirectory(src/pointcloud_math)
add_subdirectory(src/pointcloud_vis)
add_subdirectory(src/pointcloud_lcm)
add_subdirectory(src/pointcloud_filters)


