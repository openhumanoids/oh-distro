cmake_minimum_required(VERSION 2.6.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/pods.cmake)
set(POD_NAME multisense_command)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(multisense_subscribe src/multisense_subscribe.cpp)
# this is required for Conside args... grrr...
pods_use_pkg_config_packages(multisense_subscribe  bot2-core)
pods_install_executables( multisense_subscribe )
