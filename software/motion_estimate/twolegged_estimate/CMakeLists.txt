set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable -std=c++0x")

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

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics bot2-frames 
    bot2-frames_cpp visualization libfovis lcmtypes_fovis-bot2
	model-client lcm lcmtypes_drc_lcmtypes lcmtypes_bot2-core path-util)

SET(CMAKE_BUILD_TYPE debug)

################
add_library(leg-odometry  SHARED  
  src/BotTransforms.cpp  
  src/Filter.cpp 
  src/HeavyLowPassFilter.cpp 
  src/SignalTap.cpp
  src/QuaternionLib.cpp 
  src/kalman_filter.cpp
  src/IMUCompensation.cpp 
  src/OrientationComputer.cpp 
  src/VP_Mechanization.cpp 
  src/Odometry.cpp 
  src/LegOdometry_LCM_Handler.cpp 
  src/Footsteps.cpp 
  src/TwoLegOdometry.cpp
  src/KF_Models.cpp )
  
pods_use_pkg_config_packages(leg-odometry ${REQUIRED_PACKAGES}
  eigen3)
target_link_libraries (leg-odometry boost_system 
           ${REQUIRED_LIBS})
set_target_properties(leg-odometry PROPERTIES SOVERSION 1)
pods_install_libraries(leg-odometry)
pods_install_headers( src/include/leg-odometry/LegOdometry_LCM_Handler.hpp
  src/include/leg-odometry/BotTransforms.hpp
  src/include/leg-odometry/Filter.hpp
  src/include/leg-odometry/HeavyLowPassFilter.hpp
  src/include/leg-odometry/fdacoefs_1000_10_100_-20dB.h
  src/include/leg-odometry/fdacoefs_1000_5_30_-40dB.h
  src/include/leg-odometry/SignalTap.hpp
  src/include/leg-odometry/LegOdo_Parameters.hpp
  src/include/leg-odometry/Odometry.hpp  
  src/include/leg-odometry/InertialOdometry_Types.hpp
  src/include/leg-odometry/OrientationComputer.hpp
  src/include/leg-odometry/VP_Mechanization.hpp
  src/include/leg-odometry/IMUCompensation.hpp
  src/include/leg-odometry/QuaternionLib.h
  src/include/leg-odometry/Footsteps.h
  src/include/leg-odometry/TwoLegOdometry.h
  src/include/leg-odometry/TwoLegsEstimate_types.h
  src/include/leg-odometry/data_fusion_parameters.hpp
  src/include/leg-odometry/kalman_filter.hpp
  src/include/leg-odometry/KalmanFilter_Types.hpp
  src/include/leg-odometry/kf_conversion_definitions.hpp
  src/include/leg-odometry/KF_Models.hpp
  src/include/leg-odometry/KF_Tuning_Parameters.hpp
  DESTINATION leg-odometry)
pods_install_pkg_config_file(leg-odometry
  LIBS -lleg-odometry 
  VERSION 0.0.1)

add_executable(drc-legged-odometry src/test_driver.cpp)
pods_use_pkg_config_packages(drc-legged-odometry ${REQUIRED_PACKAGES} leg-odometry)
pods_install_executables(drc-legged-odometry )

add_executable(drc-foot-state src/foot_contact_classifier_main.cpp)
pods_use_pkg_config_packages(drc-foot-state ${REQUIRED_PACKAGES} leg-odometry)
pods_install_executables(drc-foot-state)

add_executable(drc-test-cppkf src/test_kalman_filter.cpp)
pods_use_pkg_config_packages(drc-test-cppkf ${REQUIRED_PACKAGES} leg-odometry)

#add_executable(testing_imu src/test_imu.cpp)
#pods_use_pkg_config_packages(testing_imu ${REQUIRED_PACKAGES} leg-odometry)

#add_executable(data-fusion-unittest ${SOURCE_FILES} src/unit_tests_imu.cpp)
#pods_use_pkg_config_packages(data-fusion-unittest ${REQUIRED_PACKAGES} leg-odometry)



#### not part of the shared library build structure yet

#add_executable(signaltap-test src/BlipFilterTest.cpp)
#pods_use_pkg_config_packages(signaltap-test eigen3)
#pods_install_executables(signaltap-test)

#add_executable(testing_isometry src/SignalTap.cpp src/QuaternionLib.cpp src/testing_isometry.cpp)
#pods_use_pkg_config_packages(testing_isometry)

