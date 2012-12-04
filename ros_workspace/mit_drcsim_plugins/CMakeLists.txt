cmake_minimum_required(VERSION 2.4.6)


include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})


include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

# listens to joint angles and publishes them (no input)
rosbuild_add_library(gazebo_ros_pub_robot_state src/gazebo_ros_pub_robot_state.cpp)
rosbuild_link_boost(gazebo_ros_pub_robot_state thread)

# drives rolling robot's base
rosbuild_add_library(fourWheeled_diffdrive_plugin src/fourWheeled_diffdrive_plugin.cpp)
rosbuild_link_boost(fourWheeled_diffdrive_plugin thread)

rosbuild_add_library(planar_hover_plugin src/planar_hover_plugin.cpp)
rosbuild_link_boost(planar_hover_plugin thread)

rosbuild_add_library(joint_actuation_plugin src/joint_actuation_plugin.cpp)
rosbuild_link_boost(joint_actuation_plugin thread)

# DRCSim DRCRobotPlugin with MIT's modifications
rosbuild_add_library(MITRobotPlugin src/MITRobotPlugin.cc)
rosbuild_link_boost(MITRobotPlugin thread)


