cmake_minimum_required(VERSION 2.4.6)

rosbuild_add_boost_directories()
rosbuild_add_executable(ros-fovis-rgbd ros_fovis_rgbd.cpp)
pods_use_pkg_config_packages(ros-fovis-rgbd lcm lcmtypes_bot2-core libfovis) 
rosbuild_link_boost(ros-fovis-rgbd signals)
