cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME test_robot_plan_publisher)
include(cmake/pods.cmake)


set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(test_robot_plan_publisher src/test_robot_plan_publisher.cpp)
pods_use_pkg_config_packages(test_robot_plan_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})


# make exeutables public
pods_install_executables(test_robot_plan_publisher)
