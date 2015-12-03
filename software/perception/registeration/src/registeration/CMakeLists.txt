set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(registeration)


##############
add_library(registration-test SHARED icp_testing_routine.cpp icp_utils.cpp cloud_accumulate.cpp clouds_io_utils.cpp)
pods_use_pkg_config_packages(registration-test path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    laser-util
    pronto_vis eigen3 bot2-frames)

target_link_libraries(registration-test pointmatcher)
set_target_properties(registration-test PROPERTIES SOVERSION 1)
pods_install_libraries(registration-test)
pods_install_headers(icp_testing_routine.hpp icp_utils.h cloud_accumulate.hpp DESTINATION registration-test)

pods_install_pkg_config_file(registration-test
    LIBS -lregistration-test 
    REQUIRES 
    VERSION 0.0.1)


################
# create an executable, and make it public
add_executable(drc-icp-simple icp_simple.cpp)
pods_use_pkg_config_packages(drc-icp-simple eigen3)
target_link_libraries(drc-icp-simple pointmatcher boost_system)
pods_install_executables(drc-icp-simple)

###############
add_executable(drc-icp-advanced icp.cpp)
pods_use_pkg_config_packages(drc-icp-advanced eigen3)
target_link_libraries(drc-icp-advanced pointmatcher boost_system)
pods_install_executables(drc-icp-advanced)

###############
add_executable(drc-lidar-accumulate lidar_accumulate.cpp  clouds_io_utils.cpp)
pods_use_pkg_config_packages(drc-lidar-accumulate registration-test bot2-frames laser-util
    path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis   eigen3)
target_link_libraries (drc-lidar-accumulate lcm pronto_math  
                        boost_system boost_thread)
pods_install_executables(drc-lidar-accumulate)

###############
add_executable(drc-icp-plot icp_plot_2D_3D.cpp)
pods_use_pkg_config_packages(drc-icp-plot path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis eigen3 bot2-frames)
target_link_libraries(drc-icp-plot registration-test lcm pronto_math 
                        boost_system boost_thread)
pods_install_executables(drc-icp-plot)


