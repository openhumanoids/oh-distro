add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)


set(PCL_LIBRARIES  pcl_io-1.6)

#set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
#	pcl_segmentation-1.6 pcl_sample_consensus-1.6
#	pcl_io-1.6)
	
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)


add_executable(porterbot-joints2frames joints2frames.cpp  )
pods_use_pkg_config_packages(porterbot-joints2frames
    path-util
    bot2-param-client
        pointcloud_lcm        pointcloud_vis        pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES} ${REQUIRED_PACKAGES})

pods_install_executables(porterbot-joints2frames )
