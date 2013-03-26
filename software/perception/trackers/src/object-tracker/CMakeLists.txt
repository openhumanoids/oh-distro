set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)


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
  color-tracker histogram-tracker
  ${PCL_LIBRARIES})
pods_install_executables(object-tracker)




