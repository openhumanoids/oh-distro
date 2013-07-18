add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map_velodyne)

set(PCL_LIBRARIES	pcl_io-1.6)
#  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
#	pcl_segmentation-1.6 pcl_sample_consensus-1.6

add_executable(drc-local-map-velodyne local_map_velodyne.cpp  )
#target_link_libraries (drc-local-map-velodyne ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-local-map-velodyne  
    lcmtypes_velodyne
    path-util
    bot2-param-client
    bot2-frames
        pointcloud_lcm        pointcloud_vis        pointcloud_math
        velodyne
        lcmtypes_drc_lcmtypes lcmtypes_visualization
       ${PCL_LIBRARIES})


pods_install_executables(drc-local-map-velodyne )
