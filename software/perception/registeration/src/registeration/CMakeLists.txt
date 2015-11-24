set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(create_octomap)


################
# create an executable, and make it public
add_executable(drc-icp-simple icp_simple.cpp)
pods_use_pkg_config_packages(drc-icp-simple eigen3)
target_link_libraries(drc-icp-simple pointmatcher boost_system)
pods_install_executables(drc-icp-simple)

add_executable(drc-icp-advanced icp.cpp)
pods_use_pkg_config_packages(drc-icp-advanced eigen3)
target_link_libraries(drc-icp-advanced pointmatcher boost_system)
pods_install_executables(drc-icp-advanced)

add_executable(drc-lidar-accumulate lidar_accumulate.cpp cloud_accumulate.cpp clouds_io_utils.cpp)
pods_use_pkg_config_packages(drc-lidar-accumulate bot2-frames laser-util
    path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis   eigen3)
target_link_libraries (drc-lidar-accumulate lcm pronto_math 
                        boost_system boost_thread)
pods_install_executables(drc-lidar-accumulate)

add_executable(drc-icp-testing icp_testing.cpp icp_utils.cpp)
pods_use_pkg_config_packages(drc-icp-testing path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis eigen3 bot2-frames)
target_link_libraries(drc-icp-testing pointmatcher lcm pronto_math 
                        boost_system boost_thread)
pods_install_executables(drc-icp-testing)
