

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_LIBRARIES pcl_common-1.6 pcl_io-1.6 
  pcl_visualization-1.6
  pcl_segmentation-1.6 pcl_filters-1.6 pcl_features-1.6)


################
add_library(circle3dlib  SHARED circle3dlib.cpp)
pods_use_pkg_config_packages(circle3dlib ${PCL_LIBRARIES}
  eigen3)
set_target_properties(circle3dlib PROPERTIES SOVERSION 1)
pods_install_libraries(circle3dlib)
pods_install_headers( circle3dlib.hpp DESTINATION circle3dlib)
pods_install_pkg_config_file(circle3dlib
  LIBS -lcircle3dlib
  VERSION 0.0.1)

