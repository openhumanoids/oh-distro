cmake_minimum_required(VERSION 2.6.0)

project(ROS2LCM_translator)

include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()

set(POD_NAME ROS2LCM_translator)
include(cmake/pods.cmake)

add_subdirectory(src)


# Old Stuff:
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#rosbuild_add_executable(TRUE_robot_state_publisher src/#TRUE_robot_state_publisher.cpp)
#pods_install_executables(TRUE_robot_state_publisher)
#pods_use_pkg_config_packages(TRUE_robot_state_publisher lcm #lcmtypes_drc_lcmtypes bot2-core) 


