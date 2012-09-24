cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tf_solver)
include(cmake/pods.cmake)

FILE(GLOB TF_SOLVER_HDRS include/tf_solver/[^.]*.hpp)
pods_install_headers(${TF_SOLVER_HDRS} DESTINATION tf_solver)


set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)
add_executable(test_tf_solver src/test_tf_solver.cpp)
pods_use_pkg_config_packages(test_tf_solver lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# create an executable, and make it public

pods_install_executables(test_tf_solver)
