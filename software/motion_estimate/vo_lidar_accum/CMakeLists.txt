cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME head-stereo-experiment)
include(cmake/pods.cmake)

set(APP_NAME vo-lidar-accum)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

add_executable(${APP_NAME}
  src/LidarAccumulator.cpp
  src/Main.cpp)
pods_use_pkg_config_packages(${APP_NAME}
  drc-utils
  opencv
  voconfig
  fovision
  maps)
pods_install_executables(${APP_NAME})
