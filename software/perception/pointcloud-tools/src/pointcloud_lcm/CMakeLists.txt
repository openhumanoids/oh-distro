################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pointcloud_lcm)

set(PCL_LIBRARIES  pcl_io-1.6)

add_library(pointcloud_lcm SHARED pointcloud_lcm.cpp)

pods_use_pkg_config_packages(pointcloud_lcm ${PCL_LIBRARIES} lcm eigen3   bot2-core  glib-2.0  )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pointcloud_lcm PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_lcm)
pods_install_headers(pointcloud_lcm.hpp DESTINATION pointcloud_tools)
pods_install_pkg_config_file(pointcloud_lcm
    LIBS -lpointcloud_lcm
    REQUIRES bot2-vis
    VERSION 0.0.1)

