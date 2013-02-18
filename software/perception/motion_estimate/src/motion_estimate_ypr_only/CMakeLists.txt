add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)


#set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
#	pcl_segmentation-1.6 pcl_sample_consensus-1.6
#	pcl_io-1.6)

#add_executable(drc-motion-estimate-imu-ypr-only local_map.cpp  )
#pods_use_pkg_config_packages(drc-motion-estimate-imu-ypr-only
#    lcmtypes_velodyne
#    path-util
#    bot2-param-client
#        pointcloud_lcm        pointcloud_vis        pointcloud_math
#	lcmtypes_drc_lcmtypes  lcmtypes_visualization
#       ${PCL_LIBRARIES})
#
#pods_install_executables(drc-motion-estimate-imu-ypr-only )

add_executable(drc-motion-estimate-imu-ypr-only imu_to_pose.cpp  )
pods_use_pkg_config_packages(drc-motion-estimate-imu-ypr-only
	lcmtypes_drc_lcmtypes)

pods_install_executables(drc-motion-estimate-imu-ypr-only )
