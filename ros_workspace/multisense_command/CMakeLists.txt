cmake_minimum_required(VERSION 2.6.0)

project(multisense_command)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(POD_NAME multisense_command)
include(cmake/pods.cmake)

add_subdirectory(src)
