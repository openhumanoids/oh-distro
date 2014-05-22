cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME robotiqhand)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

pods_install_python_script(robotiq_tcp_node python/pyrobotiqhand/robotiq_tcp_node.py)
pods_install_python_script(robotiq_command python/pyrobotiqhand/robotiq_command.py)
pods_install_python_script(robotiq_command_indiv python/pyrobotiqhand/robotiq_command_indiv.py)

