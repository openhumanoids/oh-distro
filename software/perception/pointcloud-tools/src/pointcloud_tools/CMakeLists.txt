################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pointcloud_tools)

set(PCL_LIBRARIES  pcl_io-1.6)

add_library(pointcloud_tools SHARED pointcloud_tools.cpp)

pods_use_pkg_config_packages(pointcloud_tools ${PCL_LIBRARIES} lcm eigen3   bot2-core  glib-2.0  )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pointcloud_tools PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_tools)
pods_install_headers(pointcloud_tools.hpp DESTINATION pointcloud_utils)
pods_install_pkg_config_file(pointcloud_tools
    LIBS -lpointcloud_tools
    REQUIRES bot2-vis
    VERSION 0.0.1)

