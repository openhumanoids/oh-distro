cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME robot_model_publisher)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

# build two LCM test programs
add_executable(broadcast_robot_model src/broadcast_robot_model.cpp)
pods_use_pkg_config_packages(broadcast_robot_model urdf lcm lcmtypes_drc_lcmtypes)

add_executable(robot_model_listener src/robot_model_listener.cpp)
pods_use_pkg_config_packages(robot_model_listener urdf lcm lcmtypes_drc_lcmtypes)

# make exeutables public
pods_install_executables(broadcast_robot_model robot_model_listener)
