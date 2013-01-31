cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME footstep_planner)
include(cmake/pods.cmake)


add_executable(footstep_plan_publisher src/footstep_plan_publisher.cpp)
pods_use_pkg_config_packages(footstep_plan_publisher lcm lcmtypes_drc_lcmtypes)

# make exeutables public
pods_install_executables(footstep_plan_publisher)
