cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME est_robot_state_publisher)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(est_robot_state_publisher src/est_robot_state_publisher.cpp)
pods_use_pkg_config_packages(est_robot_state_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# make exeutables public
pods_install_executables(est_robot_state_publisher)

add_executable(est_robot_state_publisher_no_sensing src/est_robot_state_publisher_no_sensing.cpp)
pods_use_pkg_config_packages(est_robot_state_publisher_no_sensing lcm lcmtypes_drc_lcmtypes)

# make exeutables public
pods_install_executables(est_robot_state_publisher_no_sensing)

add_executable(est_robot_state_publisher_no_exteroception src/est_robot_state_publisher_no_exteroception.cpp)
pods_use_pkg_config_packages(est_robot_state_publisher_no_exteroception lcm lcmtypes_drc_lcmtypes)

# make exeutables public
pods_install_executables(est_robot_state_publisher_no_exteroception)
