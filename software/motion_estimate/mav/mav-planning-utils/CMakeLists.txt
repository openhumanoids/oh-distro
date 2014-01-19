cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME mav-planning-utils)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(src/test)
