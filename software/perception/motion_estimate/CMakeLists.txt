cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME motion_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/joints2frames)

add_subdirectory(src/motion_estimate)
add_subdirectory(src/motion_estimate_scanmatcher_bolt_vertical)
add_subdirectory(src/motion_estimate_scanmatcher_righted_vertical)
add_subdirectory(src/motion_estimate_scanmatcher_rotating)

add_subdirectory(src/motion_estimate_ypr_only)


#find_package(PythonInterp REQUIRED)
#pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/scripts)

pods_install_python_script(drc-python-lcmscope scripts/python_lcmscope.py)
#INSTALL_FILES(<dir> regexp)

