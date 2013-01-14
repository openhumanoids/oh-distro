cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME qt4_widget)
include(cmake/pods.cmake)

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(src/test)
