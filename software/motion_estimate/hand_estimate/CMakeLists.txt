cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME hand_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# 
add_subdirectory(src/hand_estimate)
add_subdirectory(src/hand_tactile_viz)


# Keyboard Teleop - LCM version
pods_install_python_script(sandia-simple-grasp scripts/simple_grasp.py)
pods_install_python_script(sandia-simple-grasp-keyboard scripts/simple_grasp_keyboard_teleop.py)

pods_install_python_script(irobot-simple-grasp scripts/simple_grasp_irobot.py)


