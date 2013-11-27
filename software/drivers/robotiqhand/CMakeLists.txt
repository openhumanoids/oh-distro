cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME robotiqhand)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

pods_install_python_script(robotiq_tcp_node scripts/robotiq_tcp_node.py)
pods_install_python_script(robotiq_command scripts/robotiq_command.py)
