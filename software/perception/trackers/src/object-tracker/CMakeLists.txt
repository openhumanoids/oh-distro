set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)


################ LIBRARIES ###############################
add_library(major-plane-detect  SHARED major-plane-detect.cpp)
pods_use_pkg_config_packages(major-plane-detect ${PCL_LIBRARIES} bot2-lcmgl-client maps eigen3
     pointcloud_filters)
target_link_libraries (major-plane-detect boost_system 
           ${REQUIRED_LIBS})
set_target_properties(major-plane-detect PROPERTIES SOVERSION 1)
pods_install_libraries(major-plane-detect)
pods_install_headers( major-plane-detect.hpp DESTINATION trackers)
pods_install_pkg_config_file(major-plane-detect
  LIBS -lmajor-plane-detect
  VERSION 0.0.1)
  
add_library(color-threshold  SHARED color-threshold.cpp)
pods_use_pkg_config_packages(color-threshold 
   image_io_utils
   eigen3 opencv)
target_link_libraries (color-threshold boost_system 
           ${REQUIRED_LIBS})
set_target_properties(color-threshold PROPERTIES SOVERSION 1)
pods_install_libraries(color-threshold)
pods_install_headers( color-threshold.hpp DESTINATION trackers)
pods_install_pkg_config_file(color-threshold
  LIBS -lcolor-threshold
  VERSION 0.0.1)

  
############# INDIVIDUAL APPS #########################  
add_executable (color-tracker color_tracker.cpp )
target_link_libraries(color-tracker particle)
pods_use_pkg_config_packages(color-tracker   particle
  gsl  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  pointcloud_vis pointcloud_math opencv image_io_utils)
pods_install_executables(color-tracker)
  
add_executable (drc-major-plane-tracker major-plane-tracker-app.cpp )
pods_use_pkg_config_packages(drc-major-plane-tracker
   bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  bot2-param-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_math 
  major-plane-detect 
  ${PCL_LIBRARIES})
pods_install_executables(drc-major-plane-tracker)

  
############ COMBINED COMPLETE APP ###########################
add_executable (object-tracker object-tracker.cpp )
target_link_libraries(object-tracker particle)
pods_use_pkg_config_packages(object-tracker   particle
  gsl  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  bot2-param-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_math 
  opencv image_io_utils
  major-plane-detect 
  color-threshold tracker-histogram
  ${PCL_LIBRARIES})
pods_install_executables(object-tracker)




