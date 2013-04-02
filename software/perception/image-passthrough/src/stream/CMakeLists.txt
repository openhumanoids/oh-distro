set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
  pcl_segmentation-1.6 pcl_sample_consensus-1.6 pcl_visualization-1.6
  pcl_registration-1.6
  pcl_io-1.6)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(drc-stream stream.cpp)
target_link_libraries (drc-stream boost_system boost_thread)
pods_use_pkg_config_packages(drc-stream collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm  ${PCL_LIBRARIES})
pods_install_executables(drc-stream)
