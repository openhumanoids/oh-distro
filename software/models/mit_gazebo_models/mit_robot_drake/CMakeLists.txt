cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mit_robot_PnC)
include(cmake/pods.cmake)

#tell cmake to build these subdirectories
message(STATUS "Regenerating xacro files")
add_subdirectory(xacro)

  

