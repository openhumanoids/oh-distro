cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME control)
include(cmake/pods.cmake)

# set up matlab build
include(cmake/mex.cmake)
mex_setup()

add_subdirectory(src)

