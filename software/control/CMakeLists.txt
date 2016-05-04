cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME control)
include(cmake/pods.cmake)

# check whether snopt is available, else skip the control/matlab directory
pods_find_pkg_config(snopt_c)
if (NOT snopt_c_FOUND)
  message(WARNING "SNOPT not found, skipping control/matlab")
endif()

# check whether matlab is present, else skip the control/matlab directory
find_program(matlab matlab)
if (NOT matlab)
  message(WARNING "Matlab not found")
else (matlab)
  message(STATUS "Matlab found, setting up MEX build")
  # set up matlab build
  include(cmake/mex.cmake)
  mex_setup(REQUIRED)
endif()

find_file(MODELS models PATHS $ENV{DRC_BASE}/software)
if (NOT MODELS)
   message(FATAL_ERROR "models not found (checked ${CMAKE_INSTALL_PREFIX}/..  and $ENV{HOME} and $ENV{DRC_BASE} )")
endif()
find_file(COMMON_COMPONENTS common_components PATHS ${MODELS})
if (NOT COMMON_COMPONENTS)
   message(FATAL_ERROR "models not found (checked ${CMAKE_INSTALL_PREFIX}/..  and $ENV{HOME} and ${MODELS} )")
endif()
set(MODELS "${MODELS}" CACHE STRING "${MODELS}" )
set(COMMON_COMPONENTS "${COMMON_COMPONENTS}" CACHE STRING "${COMMON_COMPONENTS}" )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wreturn-type -Wuninitialized -Wunused-variable")

if (CMAKE_COMPILER_IS_GNUCC)
   execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpversion
                OUTPUT_VARIABLE GCC_VERSION)
   if (NOT (GCC_VERSION VERSION_GREATER 4.3 OR GCC_VERSION VERSION_EQUAL 4.3))
        message(FATAL_ERROR "requires gcc version >= 4.3")  # to support the c++0x flag below
   elseif(GCC_VERSION VERSION_LESS 4.7)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   endif()
elseif ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
 if (APPLE)
   # http://stackoverflow.com/questions/13445742/apple-and-shared-ptr
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libc++")
  endif()
endif()

include_directories(${INCLUDE_INSTALL_PATH})

add_subdirectory(src)

if (matlab)
  add_subdirectory(matlab/@QPReactiveRecoveryPlan)
endif()

add_subdirectory(contact_monitor)
add_subdirectory(fall_detector)
add_subdirectory(joint_monitor)

if (matlab)
  message(STATUS "Writing addpath_control.m and rmpath_control.m to ${CMAKE_INSTALL_PREFIX}/matlab")

  file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_control.m
  	   "function addpath_control()\n"
       "  setenv('ROS_PACKAGE_PATH',[getenv('ROS_PACKAGE_PATH'),':${MODELS}:${COMMON_COMPONENTS}']);\n"
       "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
  	   "  add_ctrl_path();\n"
  	   "  cd(wd);\n"
      )

  file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_control.m
  	   "function rmpath_control()\n"
  	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
  	   "  rm_ctrl_path();\n"
  	   "  cd(wd);\n"
      )

  message(STATUS "Writing ${CMAKE_INSTALL_PREFIX}/config/drc_control_setup.m")

  file(WRITE ${CMAKE_INSTALL_PREFIX}/config/drc_control_setup.m
  	   "function drc_control_setup()\n"
  	   "  addpath('${CMAKE_INSTALL_PREFIX}/matlab');\n"
      )

  enable_testing()
  include(CTest)

  add_test(NAME testQPReactiveRecovery
           COMMAND testQPReactiveRecovery)
  add_test(NAME testAtlasFallDetector
           COMMAND testAtlasFallDetector)
endif()

#if (NOT matlab-NOTFOUND)
#   add_test(NAME MATLAB_Unit_Tests
#   		 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#  		 COMMAND ${matlab} -nosplash -nodesktop -logfile /tmp/matlab.out -r "addpath(fullfile('${CMAKE_INSTALL_PREFIX}','matlab'));addpath_drake; addpath_control; set(0,'DefaultFigureVisible','off'); options.gui=false; options.autorun=true;options.logfile='/tmp/unit.log'; d=unitTest(options); exit(d.fail)")
#endif()

# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/
