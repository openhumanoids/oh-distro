cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME registeration)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/voconfig)
add_subdirectory(src/vofeatures)
add_subdirectory(src/voestimator)
add_subdirectory(src/fovision)


add_subdirectory(src/registeration)
add_subdirectory(src/reg_vo)
add_subdirectory(src/descript)

add_subdirectory(src/image-splitter)
