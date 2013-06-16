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

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics bot2-frames bot2-frames_cpp visualization libfovis lcmtypes_fovis-bot2)

add_executable(drc-legged-odometry src/BotTransforms.cpp src/Filter.cpp src/HeavyLowPassFilter.cpp src/SignalTap.cpp src/QuaternionLib.cpp src/InertialOdometry_Types.hpp src/IMUCompensation.cpp src/OrientationComputer.cpp src/VP_Mechanization.cpp src/Odometry.cpp src/LegOdometry_LCM_Handler.cpp src/Footsteps.cpp src/TwoLegOdometry.cpp src/test_driver.cpp)
add_executable(drc-foot-state src/BotTransforms.cpp src/Filter.cpp src/HeavyLowPassFilter.cpp src/SignalTap.cpp src/QuaternionLib.cpp src/InertialOdometry_Types.hpp src/IMUCompensation.cpp src/OrientationComputer.cpp src/VP_Mechanization.cpp src/Odometry.cpp src/LegOdometry_LCM_Handler.cpp src/Footsteps.cpp src/TwoLegOdometry.cpp src/foot_contact_classifier_main.cpp)
#add_executable(signaltap-test src/SignalTap.cpp src/BlipFilterTest.cpp)
#add_executable(testing_isometry src/Filter.cpp src/SignalTap.cpp src/QuaternionLib.cpp src/testing_isometry.cpp)
#add_executable(testing_imu src/BotTransforms.cpp src/QuaternionLib.cpp src/test_imu.cpp)
add_executable(data-fusion-unittest src/BotTransforms.cpp src/Filter.cpp src/HeavyLowPassFilter.cpp src/SignalTap.cpp src/QuaternionLib.cpp src/InertialOdometry_Types.hpp src/IMUCompensation.cpp src/OrientationComputer.cpp src/VP_Mechanization.cpp src/Odometry.cpp src/LegOdometry_LCM_Handler.cpp src/Footsteps.cpp src/TwoLegOdometry.cpp src/unit_tests_imu.cpp)

pods_use_pkg_config_packages(drc-legged-odometry model-client lcm lcmtypes_drc_lcmtypes lcmtypes_bot2-core ${REQUIRED_PACKAGES})
pods_use_pkg_config_packages(drc-foot-state model-client lcm lcmtypes_drc_lcmtypes lcmtypes_bot2-core ${REQUIRED_PACKAGES})
#pods_use_pkg_config_packages(signaltap-test eigen3)
#pods_use_pkg_config_packages(testing_isometry)
#pods_use_pkg_config_packages(testing_imu lcm lcmtypes_drc_lcmtypes lcmtypes_bot2-core)
pods_use_pkg_config_packages(data-fusion-unittest model-client lcm lcmtypes_drc_lcmtypes lcmtypes_bot2-core ${REQUIRED_PACKAGES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable -std=c++0x")


pods_install_executables(drc-legged-odometry )
pods_install_executables(drc-foot-state )
#pods_install_executables(signaltap-test)
