set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)


################ LIBRARIES ###############################
add_library(color-tracker  SHARED color-tracker.cpp)
pods_use_pkg_config_packages(color-tracker 
   image_io_utils
   eigen3 opencv)
target_link_libraries (color-tracker boost_system 
           ${REQUIRED_LIBS})
set_target_properties(color-tracker PROPERTIES SOVERSION 1)
pods_install_libraries(color-tracker)
pods_install_headers( color-tracker.hpp DESTINATION trackers)
pods_install_pkg_config_file(color-tracker
  LIBS -lcolor-tracker
  VERSION 0.0.1)

  
############# INDIVIDUAL APPS #########################  
add_executable (drc-color-tracker-app color-tracker-app.cpp )
target_link_libraries(drc-color-tracker-app  particle)
pods_use_pkg_config_packages(drc-color-tracker-app  particle
  gsl  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  pointcloud_vis pointcloud_math opencv image_io_utils)
pods_install_executables(drc-color-tracker-app )
