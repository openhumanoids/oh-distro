cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME motion_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/joints2frames)

add_subdirectory(src/voconfig)
add_subdirectory(src/vofeatures)
add_subdirectory(src/voestimator)
add_subdirectory(src/fovision)

add_subdirectory(src/registeration)
add_subdirectory(src/tools)

pods_install_python_script(drc-lcmscope scripts/python_lcmscope.py)
pods_install_python_script(drc-bandwidth-scope scripts/bandwidth_scope.py)



## Older Depreciated Stuff for Laser and Rotating Laser:
#add_subdirectory(src/motion_estimate)
#add_subdirectory(src/motion_estimate_scanmatcher_bolt_vertical)
#add_subdirectory(src/motion_estimate_scanmatcher_righted_vertical)
#add_subdirectory(src/motion_estimate_scanmatcher_rotating)
#add_subdirectory(src/motion_estimate_ypr_only)
