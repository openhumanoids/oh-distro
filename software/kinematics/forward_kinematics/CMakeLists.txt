cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME robot_state_publisher)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

# Create a shared library 
add_library(forward-kinematics SHARED src/treefksolverposfull_recursive.cpp src/tf_publisher.cpp)

FILE(GLOB ROBOT_STATE_PUBLISHER_HDRS include/robot_state_publisher/[^.]*.hpp)
pods_install_headers(${ROBOT_STATE_PUBLISHER_HDRS} DESTINATION robot_state_publisher)

# make the library public
pods_install_libraries(forward-kinematics)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES orocos-kdl)
pods_use_pkg_config_packages(forward-kinematics lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(forward-kinematics
    CFLAGS
    LIBS -lforward-kinematics
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# build two LCM test programs
add_executable(robot_state_publisher src/robot_state_publisher.cpp)

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)
pods_use_pkg_config_packages(robot_state_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

add_executable(test_forward_kinematics_solver src/test_forward_kinematics_solver.cpp)
pods_use_pkg_config_packages(test_forward_kinematics_solver lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

add_executable(robot_state_listener src/robot_state_listener.cpp)
pods_use_pkg_config_packages(robot_state_listener lcm lcmtypes_drc_lcmtypes)

# make exeutables public
pods_install_executables(robot_state_publisher robot_state_listener test_forward_kinematics_solver)
