cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME motion_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


add_subdirectory(src/joints2frames)
add_subdirectory(src/robot_state_publisher)
add_subdirectory(src/state_sync)
add_subdirectory(src/utime_publisher)
add_subdirectory(src/pseudo_pose_publisher)

add_subdirectory(src/frames_changer)
