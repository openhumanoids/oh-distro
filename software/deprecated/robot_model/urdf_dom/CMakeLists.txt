cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME urdf_dom)
include(cmake/pods.cmake)

#tell cmake to build these subdirectories
add_subdirectory(urdf_interface)
add_subdirectory(urdf_parser)

#add_subdirectory(src)
#add_subdirectory(src/test)
