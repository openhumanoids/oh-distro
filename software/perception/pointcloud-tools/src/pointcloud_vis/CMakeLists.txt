################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pointcloud_vis)

set(PCL_LIBRARIES  pcl_io-1.6 )

add_library(pointcloud_vis SHARED pointcloud_vis.cpp)

pods_use_pkg_config_packages(pointcloud_vis ${PCL_LIBRARIES}  lcm eigen3   bot2-core  
	glib-2.0  lcmtypes_pointcloud-tools )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pointcloud_vis PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_vis)
pods_install_headers(pointcloud_vis.hpp DESTINATION pointcloud_tools)
pods_install_pkg_config_file(pointcloud_vis
    LIBS -lpointcloud_vis 
    REQUIRES bot2-vis pcl_io-1.6 lcmtypes_visualization pointcloud_math
    VERSION 0.0.1)

