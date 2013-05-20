cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME qt4_widget_gfe)
set(CMAKE_MODULE_PATH /home/tmhoward/software/packages/install/share/cmake-2.8/Modules)
include(cmake/pods.cmake)

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(src/test)
