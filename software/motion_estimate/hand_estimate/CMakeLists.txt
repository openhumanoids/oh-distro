cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME hand_estimate)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build two LCM test programs
add_executable(sandia_hand_tactile_processor src/sandia_hand_tactile_processor.cpp)
pods_use_pkg_config_packages(sandia_hand_tactile_processor lcm lcmtypes_${POD_NAME})

# make exeutables public
pods_install_executables(sandia_hand_tactile_processor)
