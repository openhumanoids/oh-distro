cmake_minimum_required(VERSION 2.4.6)

rosbuild_add_boost_directories()
rosbuild_add_executable(ros-stereo2lcm ros_stereo2lcm.cpp)
pods_use_pkg_config_packages(ros-stereo2lcm lcm lcmtypes_bot2-core libfovis) 
rosbuild_link_boost(ros-stereo2lcm signals)
