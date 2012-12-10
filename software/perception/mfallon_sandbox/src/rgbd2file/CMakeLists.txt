add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(rgbd2file)


set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
	pcl_segmentation-1.6 pcl_sample_consensus-1.6
	pcl_io-1.6)


add_executable(drc-rgbd2file rgbd2file.cpp  )
pods_use_pkg_config_packages(drc-rgbd2file  
    bot2-lcmgl-client eigen3
		lcmtypes_kinect 
	pointcloud_vis        pointcloud_math 
	pointcloud_lcm       ${PCL_LIBRARIES}
		lcmtypes_visualization)


pods_install_executables(drc-rgbd2file )
