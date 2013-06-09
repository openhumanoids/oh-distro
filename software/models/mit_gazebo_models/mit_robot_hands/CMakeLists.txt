cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mit_robot_hands)
include(cmake/pods.cmake)

#tell cmake to build these subdirectories
#add_subdirectory(xacro)
add_subdirectory(drake_urdfs/xacro)
#add_subdirectory(floatingbase_urdfs/xacro)
#add_subdirectory(visual_contact_spheres/xacro)
message(STATUS "Regenerating xacro files")
  

