add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_LIBRARIES pcl_common-1.6 pcl_io-1.6 pcl_visualization-1.6
  pcl_segmentation-1.6 pcl_filters-1.6 pcl_features-1.6)

#set(PCL_DIR "/home/hammoud/drc/software/externals/pcl_drc/pcl-1.6.0")

#set(PCL_DIR "/home/hammoud/drc/software/perception/rhammoud_sandbox/circle3d/pcl_trunk1.7/trunk/"); 

#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})

add_executable(cylinder_segmentation cylinder_segmentation.cpp  )
pods_use_pkg_config_packages(cylinder_segmentation  ${PCL_LIBRARIES}
   lcmtypes_drc_lcmtypes
   bot2-lcmgl-client )

target_link_libraries(cylinder_segmentation boost_thread)
#include_directories(cylinder_segmentation vtk-5.6)
pods_install_executables(cylinder_segmentation )
