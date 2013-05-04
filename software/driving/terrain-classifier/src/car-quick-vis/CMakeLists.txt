set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-car-quick-vis car-quick-vis.cpp)
pods_use_pkg_config_packages(drc-car-quick-vis collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm)
pods_install_executables(drc-car-quick-vis)
