cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME residual_detector)
include(cmake/pods.cmake)

add_subdirectory(src)