cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME CollierTools)
include(cmake/pods.cmake)

# Eigen3 is needed
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/misc)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build two LCM test programs
add_executable(CollierTools_unit_test src/FusionStabLoop.cpp src/Filter.cpp src/unit_test.cpp)
pods_use_pkg_config_packages(CollierTools_unit_test lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# make exeutables public
#pods_install_executables()
