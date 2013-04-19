cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME twolegged_estimate)
include(cmake/pods.cmake)

# Eigen3 is needed
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/misc)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics bot2-frames bot2-frames_cpp visualization)

add_executable(drc-legged-odometry src/SignalTap.cpp src/QuaternionLib.cpp src/LegOdometry_LCM_Handler.cpp src/Footsteps.cpp src/TwoLegOdometry.cpp src/test_driver.cpp)
pods_use_pkg_config_packages(drc-legged-odometry model-client lcm lcmtypes_drc_lcmtypes lcmtypes_bot2-core ${REQUIRED_PACKAGES})

pods_install_executables(drc-legged-odometry )

