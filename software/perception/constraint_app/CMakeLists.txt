cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME constraint_app)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/constraint_app)
add_subdirectory(src/matlab_mex)
add_subdirectory(src/test)

enable_testing()
find_program(matlab matlab)
if ( matlab )
   add_test(NAME constraint_app_unit_tests
   		 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		 COMMAND ./testing/test_pf.sh ${matlab})
  		 #COMMAND ${matlab} -nosplash -r "addpath('${CMAKE_INSTALL_PREFIX}/matlab'); addpath_drake; options.gui=false; options.autorun=true;options.logfile='unitTest.log';options.cdash='cdash.xml'; d=unitTest(options); exit(d.fail)")
endif()
