add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)


set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
	pcl_segmentation-1.6 pcl_sample_consensus-1.6
	pcl_io-1.6)

add_executable(drc-map-store map_store.cpp  )
#target_link_libraries (drc-map-store-velodyne ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-map-store
    lcmtypes_velodyne
    path-util
    bot2-param-client
        pointcloud_lcm        pointcloud_vis        pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES})

pods_install_executables(drc-map-store )
