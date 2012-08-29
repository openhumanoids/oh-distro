cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_examples)

set(PCL_LIBRARIES  pcl_io-1.6)

add_executable(drc-local-map local_map.cpp   )
target_link_libraries (drc-local-map ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-local-map  
        pointcloud_lcm        pointcloud_vis        pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
        ${PCL_LIBRARIES})

pods_install_executables(drc-local-map )
