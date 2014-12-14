cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME valkyrie)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


pods_install_python_script(val-send-com-height python/send_com_height.py)
pods_install_python_script(val-send-pause python/send_pause.py)
pods_install_python_script(val-send-arm-test python/send_arm_test.py)
