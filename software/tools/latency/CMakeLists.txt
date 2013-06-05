cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME blah)
include(cmake/pods.cmake)

add_subdirectory(src/latency)
add_subdirectory(src/frequency)
add_subdirectory(src/printf-cache)
add_subdirectory(src/control-hangup)

# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

# install a script "hello-python" that runs the hello.main python module
# This script gets installed to ${CMAKE_INSTALL_PREFIX}/bin/hello-python
# and automatically sets the correct python path.

pods_install_python_script(drc_latency latency.drc_latency)
pods_install_python_script(drc_latency_ros latency.latency_ros)
pods_install_python_script(drc_latency_translator latency.latency_translator)
pods_install_python_script(drc_latency_odometry latency.latency_odometry)


