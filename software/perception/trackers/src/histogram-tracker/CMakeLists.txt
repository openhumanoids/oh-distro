set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-histogram-tracker 
  histogram-tracker.cpp
  HistTracker.cpp)
pods_use_pkg_config_packages(drc-histogram-tracker lcm 
  bot2-param-client bot2-lcmgl-client
  lcmtypes_tld-tracker
  bot2-frames  bot2-frames_cpp
  opencv  image_io_utils
  pointcloud_filters  pointcloud_vis pointcloud_lcm)
pods_install_executables(drc-histogram-tracker)
