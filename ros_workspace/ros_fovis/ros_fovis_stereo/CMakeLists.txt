cmake_minimum_required(VERSION 2.4.6)

rosbuild_add_boost_directories()
rosbuild_add_executable(ros-fovis-stereo ros_fovis_stereo.cpp)
pods_use_pkg_config_packages(ros-fovis-stereo lcm lcmtypes_bot2-core libfovis
		bot2-param-client) 
rosbuild_link_boost(ros-fovis-stereo signals)
