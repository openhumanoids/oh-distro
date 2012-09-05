cmake_minimum_required(VERSION 2.4.6)

rosbuild_add_boost_directories()
rosbuild_add_executable(ros-stereo2lcm-fuerte ros_stereo2lcm_fuerte.cpp)
pods_use_pkg_config_packages(ros-stereo2lcm-fuerte lcm lcmtypes_bot2-core
        lcmtypes_drc_lcmtypes
		bot2-param-client) 
rosbuild_link_boost(ros-stereo2lcm-fuerte signals)



rosbuild_add_executable(ros-stereo2lcm-fuerte-single ros_stereo2lcm_fuerte_single.cpp)
pods_use_pkg_config_packages(ros-stereo2lcm-fuerte-single lcm lcmtypes_bot2-core
        lcmtypes_drc_lcmtypes
		bot2-param-client) 
rosbuild_link_boost(ros-stereo2lcm-fuerte-single signals)
