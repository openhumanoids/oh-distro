cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME motion_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/voconfig)
add_subdirectory(src/vofeatures)
add_subdirectory(src/voestimator)
add_subdirectory(src/fovision)

add_subdirectory(src/registeration)

add_subdirectory(src/motion_estimate_ypr_only)
add_subdirectory(src/blacken_image)

add_subdirectory(src/lidar-odometry)

pods_install_python_script(spoof-driving-posture scripts/spoof_driving_posture.py)
