cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME state_estimate)
include(cmake/pods.cmake)

SET(CMAKE_BUILD_TYPE Debug)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


add_subdirectory(src/basic_legodo)
add_subdirectory(src/state_estimate)
add_subdirectory(src/mfallon_sandbox) # master build was broken


pods_install_python_script(se-bdi-imu-individually scripts/republish_bdi_imu_individually.py)



file(COPY scripts/se-batch-process.sh DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

