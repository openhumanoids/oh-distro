set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


set(PCL_LIBRARIES  pcl_filters-1.7)


add_executable(drc-vehicle-tracker vehicle-tracker.cpp)
pods_use_pkg_config_packages(drc-vehicle-tracker collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES})
target_link_libraries (drc-vehicle-tracker boost_system boost_thread)
pods_install_executables(drc-vehicle-tracker)
