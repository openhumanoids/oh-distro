cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME perception-opencv-utils)
include(cmake/pods.cmake)


find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv)

#tell cmake to build these subdirectories
add_subdirectory(src)
