cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME local_map)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

add_subdirectory(src/local_map)
add_subdirectory(src/local_map_velodyne)
add_subdirectory(src/velodyne2pcd)

add_subdirectory(src/local_segment)
add_subdirectory(src/gen-mesh)


