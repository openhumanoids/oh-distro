cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME py_drake_utils)
include(cmake/pods.cmake)

# require python
find_package(PythonInterp REQUIRED)

# install all python files in the python/ subdirectory
pods_install_python_packages(${CMAKE_CURRENT_SOURCE_DIR}/python)

pods_install_python_script(drc-py-mat-tester mat_tester.main)

# pull in the pods matlab macros
include(cmake/matlab_pods.cmake)
# create a script for adding the py_drake_utils/matlab folder to the matlab path
pods_install_matlab_path(${POD_NAME})

enable_testing()
find_program(matlab matlab)
if (NOT matlab-NOTFOUND)
	add_test(NAME MATLAB_Unit_Tests
			 WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
			 COMMAND ${matlab} -nosplash -nodesktop -logfile /tmp/py_drake_utils.out -r "addpath_control; addpath_py_drake_utils; runPyDrakeUtilsTests; exit(0)")
 endif()

