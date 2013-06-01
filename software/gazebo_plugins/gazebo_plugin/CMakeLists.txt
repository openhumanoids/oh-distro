cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME gazebo_plugin)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

#add_subdirectory(src/hokuyospin_plugin)
#add_subdirectory(src/oracle_plugin)

add_subdirectory(src/rateset_plugin)

# pops the oracle collections over to the affordance_collection message
pods_install_python_script(drc-oracle2collection scripts/oracle2collection.py)

