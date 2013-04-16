cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME trackers)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/major-plane-detect)
add_subdirectory(src/color-tracker)
add_subdirectory(src/icp-tracker)
add_subdirectory(src/histogram-tracker)
add_subdirectory(src/tld-tracker)

# particle filter based:
add_subdirectory(src/object-tracker)

add_subdirectory(src/vehicle-tracker)
add_subdirectory(src/feature-tracker)

add_subdirectory(src/tools)

#pods_install_python_script(drc-send-tracker-command scripts/drc-send-tracker-command.py)

