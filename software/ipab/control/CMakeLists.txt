cmake_minimum_required(VERSION 2.6.0)

# set up pods
set(POD_NAME ipab_control)
include(cmake/pods.cmake)

# set up matlab build
include(cmake/mex.cmake)
mex_setup(REQUIRED)

find_file(ROS_MODEL_PKG mit_drcsim_scripts PATHS ${CMAKE_INSTALL_PREFIX}/.. $ENV{HOME} $ENV{DRC_BASE}/ros_workspace)
if (NOT ROS_MODEL_PKG)
   message(FATAL_ERROR "mit_drcsim_scripts not found (checked ${CMAKE_INSTALL_PREFIX}/.. and $ENV{HOME} and $ENV{DRC_BASE}/ros_workspace )")
endif()
find_file(MODELS models PATHS $ENV{DRC_BASE}/software)
if (NOT MODELS)
   message(FATAL_ERROR "models not found (checked ${CMAKE_INSTALL_PREFIX}/..  and $ENV{HOME} and $ENV{DRC_BASE} )")
endif()
find_file(COMMON_COMPONENTS common_components PATHS ${MODELS})
if (NOT COMMON_COMPONENTS)
   message(FATAL_ERROR "models not found (checked ${CMAKE_INSTALL_PREFIX}/..  and $ENV{HOME} and ${MODELS} )")
endif()
set(ROS_MODEL_PKG "${ROS_MODEL_PKG}" CACHE STRING "${ROS_MODEL_PKG}" )
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


message(STATUS "Writing addpath_ipab_control.m and rmpath_ipab_control.m to ${CMAKE_INSTALL_PREFIX}/matlab")

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/addpath_ipab_control.m
	   "function addpath_ipab_control()\n"
     "  setenv('ROS_PACKAGE_PATH',[getenv('ROS_PACKAGE_PATH'),':${ROS_MODEL_PKG}:${MODELS}:${COMMON_COMPONENTS}']);\n"
     "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  add_ipab_ctrl_path();\n"
	   "  cd(wd);\n"
    )

file(WRITE ${CMAKE_INSTALL_PREFIX}/matlab/rmpath_ipab_control.m
	   "function rmpath_ipab_control()\n"
	   "  wd = cd('${CMAKE_CURRENT_SOURCE_DIR}');\n"
	   "  rm_ipab_ctrl_path();\n"
	   "  cd(wd);\n"
    )

message(STATUS "Writing ${CMAKE_INSTALL_PREFIX}/config/drc_ipab_control_setup.m")

file(WRITE ${CMAKE_INSTALL_PREFIX}/config/drc_ipab_control_setup.m
	   "function drc_ipab_control_setup()\n"
	   "  addpath('${CMAKE_INSTALL_PREFIX}/matlab');\n"
    )

enable_testing()
include(CTest)

#find_program(matlab matlab)
#if (NOT matlab-NOTFOUND)
#   add_test(NAME MATLAB_Unit_Tests
#   		 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
#  		 COMMAND ${matlab} -nosplash -nodesktop -logfile /tmp/matlab.out -r "addpath(fullfile('${CMAKE_INSTALL_PREFIX}','matlab'));addpath_drake; addpath_ipab_control; set(0,'DefaultFigureVisible','off'); options.gui=false; options.autorun=true;options.logfile='/tmp/unit.log'; d=unitTest(options); exit(d.fail)")
#endif()

# todo: build doxygen here (and remove it from the Makefile)
# http://majewsky.wordpress.com/2010/08/14/tip-of-the-day-cmake-and-doxygen/
