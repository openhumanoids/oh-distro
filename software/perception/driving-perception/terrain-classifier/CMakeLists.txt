cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME terrain-classifier)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(drc-terrain-classifier src/main.cpp)

pods_use_pkg_config_packages(drc-terrain-classifier bot2-core bot2-frames opencv perception-opencv-utils image_io_utils bot2-lcmgl-client) 

pods_install_executables(drc-terrain-classifier)
