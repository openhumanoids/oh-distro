set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)


set(PCL_LIBRARIES  pcl_io-1.6)

add_executable(drc-terrain-classify terrain-classify.cpp  )
pods_use_pkg_config_packages(drc-terrain-classify
    path-util
    bot2-param-client bot2-lcmgl-client eigen3
        pointcloud_lcm        pointcloud_vis        pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES})

pods_install_executables(drc-terrain-classify )




add_executable(drc-simple-classify simple-classify.cpp  )
#target_link_libraries (drc-simple-classify-velodyne ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-simple-classify
  lcmtypes_velodyne
  path-util
  bot2-param-client
  pointcloud_lcm        pointcloud_vis        pointcloud_math
  lcmtypes_drc_lcmtypes  lcmtypes_visualization 
  bot2-frames
  bot2-frames_cpp
  ${PCL_LIBRARIES})

pods_install_executables(drc-simple-classify )
