add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_LIBRARIES pcl_common-1.6 pcl_io-1.6 
  pcl_visualization-1.6
  pcl_segmentation-1.6 pcl_filters-1.6 pcl_features-1.6)

#set(PCL_DIR "/home/hammoud/drc/software/externals/pcl_drc/pcl-1.6.0")

#set(PCL_DIR "/home/hammoud/drc/software/perception/rhammoud_sandbox/circle3d/#pcl_trunk1.7/trunk/"); 

#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

add_executable(circle3d circle3d.cpp  )
pods_use_pkg_config_packages(circle3d  ${PCL_LIBRARIES}
   bot2-lcmgl-client circle3dlib )

target_link_libraries(circle3d boost_thread)
pods_install_executables(circle3d )
