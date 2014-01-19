cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mav-kmcl)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/gpf-rgbd-lib)
add_subdirectory(src/mav_est_fovis)
#add_subdirectory(src/mav_est_legodo)
add_subdirectory(src/mav_kmcl)

