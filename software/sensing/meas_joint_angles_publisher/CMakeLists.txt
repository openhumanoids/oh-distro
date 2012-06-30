cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME meas_joint_angles_publisher)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

set(REQUIRED_PACKAGES urdf)
# build two LCM test programs
add_executable(meas_joint_angles_publisher src/meas_joint_angles_publisher.cpp)
pods_use_pkg_config_packages(meas_joint_angles_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

add_executable(meas_joint_angles_listener src/meas_joint_angles_listener.cpp)
pods_use_pkg_config_packages(meas_joint_angles_listener lcm lcmtypes_drc_lcmtypes)

add_executable(sim_meas_joint_angles_publisher src/sim_meas_joint_angles_publisher.cpp)
pods_use_pkg_config_packages(sim_meas_joint_angles_publisher lcm lcmtypes_drc_lcmtypes)

# make executables public
pods_install_executables(meas_joint_angles_publisher meas_joint_angles_listener sim_meas_joint_angles_publisher)
