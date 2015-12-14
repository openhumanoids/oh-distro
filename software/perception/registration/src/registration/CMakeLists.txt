set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(registration)


##############
add_library(icp-registration SHARED icp_testing_routine.cpp icp_3Dreg_and_plot.cpp icp_utils.cpp cloud_accumulate.cpp clouds_io_utils.cpp)
pods_use_pkg_config_packages(icp-registration path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    laser-util
    pronto_vis eigen3 bot2-frames)

target_link_libraries(icp-registration pointmatcher)
set_target_properties(icp-registration PROPERTIES SOVERSION 1)
pods_install_libraries(icp-registration)
pods_install_headers(icp_3Dreg_and_plot.hpp icp_testing_routine.hpp icp_utils.h cloud_accumulate.hpp clouds_io_utils.h DESTINATION icp-registration)

pods_install_pkg_config_file(icp-registration
    LIBS -licp-registration 
    REQUIRES 
    VERSION 0.0.1)

###############
add_executable(drc-lidar-accumulate lidar_accumulate.cpp  clouds_io_utils.cpp)
pods_use_pkg_config_packages(drc-lidar-accumulate icp-registration bot2-frames laser-util
    path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis   eigen3)
target_link_libraries (drc-lidar-accumulate lcm pronto_math  
                        boost_system boost_thread)
pods_install_executables(drc-lidar-accumulate)

###############
add_executable(drc-icp-plot icp_registration_app.cpp icp_3Dreg_and_plot.cpp)
pods_use_pkg_config_packages(drc-icp-plot path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis eigen3 bot2-frames)
target_link_libraries(drc-icp-plot icp-registration lcm pronto_math 
                        boost_system boost_thread)
pods_install_executables(drc-icp-plot)


