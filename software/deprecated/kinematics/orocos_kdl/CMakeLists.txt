cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME orocos_kdl)
include(cmake/pods.cmake)

add_definitions(-DEIGEN2_SUPPORT)

#tell cmake to build these subdirectories
add_subdirectory(orocos_kinematics_dynamics/orocos_kdl/src)



