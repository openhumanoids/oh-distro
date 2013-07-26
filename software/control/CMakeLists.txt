cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME control)
include(cmake/pods.cmake)

# set up matlab build
include(cmake/mex.cmake)
mex_setup()

find_file(ROS_MODEL_PKG mit_drcsim_scripts PATHS ${CMAKE_INSTALL_PREFIX}/.. $ENV{HOME})
message(STATUS ${ROS_MODEL_PKG})

add_subdirectory(src)

message(STATUS "Writing addpath_control.m and rmpath_control.m to ${CMAKE_INSTALL_PREFIX}/matlab") 
file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_control.m
	   "function addpath_control()\n"
	   "  setenv('ROS_PACKAGE_PATH',[getenv('ROS_PACKAGE_PATH'),':${ROS_MODEL_PKG}']);\n"
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


enable_testing()
find_program(matlab matlab)
if (NOT matlab-NOTFOUND)
   add_test(NAME MATLAB_Unit_Tests 
   		 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  		 COMMAND ${matlab} -nosplash -r "addpath_drake; addpath_control; options.gui=false; options.autorun=true;options.logfile='.unitTest.log'; d=unitTest(options); exit(d.fail)")
endif()

# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/


