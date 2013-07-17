cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME control)
include(cmake/pods.cmake)

# set up matlab build
include(cmake/mex.cmake)
mex_setup()

add_subdirectory(src)

message(STATUS "Writing addpath_control.m and rmpath_control.m to ${CMAKE_INSTALL_PREFIX}/matlab") 
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_control.m
	   "function addpath_control()\n"
       "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  addpath_control();\n"
	   "  cd(wd);\n"
    )

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_control.m
	   "function rmpath_control()\n"
	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  rmpath_control();\n"
	   "  cd(wd);\n"
    )


# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/


