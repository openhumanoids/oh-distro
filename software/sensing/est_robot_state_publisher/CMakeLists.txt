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
pods_use_pkg_config_packages(est_robot_state_publisher   model-client lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})
pods_install_executables(est_robot_state_publisher)


# Tool to recreate EST_ROBOT_STATE from  minimal version on the base station
add_executable(minimal_to_est_robot_state src/minimal_to_est_robot_state.cpp)
pods_use_pkg_config_packages(minimal_to_est_robot_state   model-client lcm 
          bot2-param-client lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})
pods_install_executables(minimal_to_est_robot_state)


