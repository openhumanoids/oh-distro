set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-point-head point-head.cpp)
pods_use_pkg_config_packages(drc-point-head collision_object_gfe collision_detector collision_object lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm)
pods_install_executables(drc-point-head)
