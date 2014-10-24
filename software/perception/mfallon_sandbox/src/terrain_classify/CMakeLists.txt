set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)


set(PCL_LIBRARIES  pcl_io-1.7)

add_executable(drc-terrain-classify terrain-classify.cpp  )
target_link_libraries(drc-terrain-classify boost_system)
pods_use_pkg_config_packages(drc-terrain-classify
    path-util
    bot2-param-client bot2-lcmgl-client eigen3
        pronto_lcm        pronto_vis        pronto_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES})

pods_install_executables(drc-terrain-classify )




add_executable(drc-simple-classify simple-classify.cpp  )
target_link_libraries(drc-simple-classify boost_system)
#target_link_libraries (drc-simple-classify-velodyne ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-simple-classify
  path-util
  bot2-param-client
  pronto_lcm        pronto_vis        pronto_math
  lcmtypes_drc_lcmtypes  lcmtypes_visualization 
  bot2-frames
  bot2-frames_cpp
  ${PCL_LIBRARIES})

pods_install_executables(drc-simple-classify )
