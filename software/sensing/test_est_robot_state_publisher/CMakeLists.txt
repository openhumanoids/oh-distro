cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME test_est_robot_state_publisher)
include(cmake/pods.cmake)

set(REQUIRED_PACKAGES urdf)

# create an executable, and make it public
add_executable(test_est_robot_state_publisher src/test_est_robot_state_publisher.cpp)
pods_use_pkg_config_packages(test_est_robot_state_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

pods_install_executables(test_est_robot_state_publisher)
