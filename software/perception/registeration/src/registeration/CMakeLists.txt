set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(create_octomap)


################
# create an executable, and make it public
add_executable(drc-icp-simple icp_simple.cpp)
pods_use_pkg_config_packages(drc-icp-simple eigen3)
target_link_libraries(drc-icp-simple pointmatcher boost_system)
pods_install_executables(drc-icp-simple)


add_executable(drc-lidar-accumulate lidar_accumulate.cpp cloud_accumulate.cpp)
pods_use_pkg_config_packages(drc-lidar-accumulate bot2-frames laser-util
    path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis   eigen3)
target_link_libraries (drc-lidar-accumulate lcm pronto_math 
                        boost_system boost_thread)
pods_install_executables(drc-lidar-accumulate)
