cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME tools)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

# require python
#find_package(PythonInterp REQUIRED)
# install all python files in the python/ subdirectory
#pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/scripts)


file(COPY scripts/drc-svn-stat.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

pods_install_python_script(drc-scope-motionest scripts/python_lcmscope.py)
pods_install_python_script(drc-scope-bandwidth scripts/bandwidth_scope.py)
pods_install_python_script(drc-scope-jointrate scripts/jointrate_scope.py)
pods_install_python_script(drc-set-sim-rate scripts/send_a_rateset.py)
pods_install_python_script(drc-init-laser scripts/init_laser.py)

#
pods_install_python_script(drc_latency scripts/drc_latency.py)

