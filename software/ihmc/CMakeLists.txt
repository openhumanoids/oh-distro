cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME ihmc)
include(cmake/pods.cmake)
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/ihmc_step)

pods_install_python_script(drc-ihmc-step-translator ihmc_step.main_ihmc)
