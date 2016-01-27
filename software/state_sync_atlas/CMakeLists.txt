cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME state_sync_atlas)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build two LCM test programs
add_executable(send_message src/send_message.c)
pods_use_pkg_config_packages(send_message lcm lcmtypes_${POD_NAME})

add_executable(listener src/listener.c)
pods_use_pkg_config_packages(listener lcm lcmtypes_${POD_NAME})

# make exeutables public
pods_install_executables(send_message listener)
