cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME ipab_control)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(hello src/hello.c)

pods_install_executables(hello)


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

add_test(NAME hello
         COMMAND hello)

