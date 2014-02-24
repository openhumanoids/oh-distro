set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

#add_executable(drc-tags affordance_tags.cpp)
#pods_use_pkg_config_packages(drc-tags lcm 
#  bot2-param-client bot2-lcmgl-client
#  bot2-frames  bot2-frames_cpp
#  pointcloud_vis pointcloud_lcm
#  apriltags opencv image_io_utils affordance)
#pods_install_executables(drc-tags)




add_executable(drc-tags fk-tag.cpp)
pods_use_pkg_config_packages(drc-tags lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm
  apriltags opencv image_io_utils affordance)
pods_install_executables(drc-tags)




add_executable(drc-hand-camera-tag hand_camera_tag.cpp)
pods_use_pkg_config_packages(drc-hand-camera-tag lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm
  apriltags opencv image_io_utils affordance)
pods_install_executables(drc-hand-camera-tag)
