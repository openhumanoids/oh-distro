cmake_minimum_required(VERSION 2.6.0)

project(ros_fovis)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME ros_fovis)
include(cmake/pods.cmake)
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

add_subdirectory(ros_fovis_rgbd)
add_subdirectory(ros_fovis_stereo)
add_subdirectory(ros_stereo2lcm)
add_subdirectory(ros_stereo2lcm_fuerte)
