cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME utils)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
set(PROJECT_NAME multisense-extra)
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/multisense-glview)
add_subdirectory(src/multisense-renderer)
add_subdirectory(src/project-lidar)
add_subdirectory(src/multisense_image_utils)
add_subdirectory(src/multisense_utils)
add_subdirectory(src/test)
