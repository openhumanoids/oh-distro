cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tools)
include(cmake/pods.cmake)

# require python
#find_package(PythonInterp REQUIRED)
# install all python files in the python/ subdirectory
#pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)




pods_install_python_script(drc-lcmscope scripts/python_lcmscope.py)
pods_install_python_script(drc-bandwidth-scope scripts/bandwidth_scope.py)
#pods_install_python_script(drc-network-trigger scripts/send_a_trigger.py)
pods_install_python_script(drc-set-sim-rate scripts/send_a_rateset.py)

