set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


set(PCL_LIBRARIES  pcl_filters-1.6)


add_executable(drc-box-filter box_filter.cpp)
pods_use_pkg_config_packages(drc-box-filter collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES})
pods_install_executables(drc-box-filter)
