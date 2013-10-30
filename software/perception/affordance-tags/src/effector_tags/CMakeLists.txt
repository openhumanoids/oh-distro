set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-effector-tags effector_tags.cpp)
pods_use_pkg_config_packages(drc-effector-tags lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm
  apriltags opencv image_io_utils affordance)
pods_install_executables(drc-effector-tags)
