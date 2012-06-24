cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME joint_state_publisher)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

set(REQUIRED_PACKAGES urdf)
# build two LCM test programs
add_executable(joint_state_publisher src/joint_state_publisher.cpp)
pods_use_pkg_config_packages(joint_state_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

add_executable(joint_state_listener src/joint_state_listener.cpp)
pods_use_pkg_config_packages(joint_state_listener lcm lcmtypes_drc_lcmtypes)

# make executables public
#pods_install_executables(joint_state_publisher)
pods_install_executables(joint_state_publisher joint_state_listener)
