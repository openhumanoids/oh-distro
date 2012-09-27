add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_LIBRARIES pcl_common-1.6 pcl_io-1.6 pcl_visualization-1.6)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(drc-pcl-pcd-grabber pcl_pcd_grabber.cpp  )
pods_use_pkg_config_packages(drc-pcl-pcd-grabber  ${PCL_LIBRARIES})

target_link_libraries(drc-pcl-pcd-grabber boost_thread boost_system boost_filesystem)
include_directories(drc-pcl-pcd-grabber vtk-5.6)
#pods_use_pkg_config_packages(pointcloud_lcm  lcm eigen3
#   bot2-core  glib-2.0  
#   lcmtypes_kinect )
pods_install_executables(drc-pcl-pcd-grabber )
