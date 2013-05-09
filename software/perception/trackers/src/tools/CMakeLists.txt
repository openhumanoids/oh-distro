set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


set(PCL_LIBRARIES  pcl_filters-1.6 pcl_features-1.6 pcl_surface-1.6   pcl_io-1.6)
### needed to read obj:
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(drc-box-filter box_filter.cpp)
pods_use_pkg_config_packages(drc-box-filter collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES})
pods_install_executables(drc-box-filter)


add_executable(drc-car-clouds car-clouds.cpp)
pods_use_pkg_config_packages(drc-car-clouds collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES})
pods_install_executables(drc-car-clouds)

add_executable(drc-firehose-clouds firehose-clouds.cpp)
target_link_libraries(drc-firehose-clouds boost_system  boost_random)

pods_use_pkg_config_packages(drc-firehose-clouds collision_object_gfe collision_detector collision_object lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES})
pods_install_executables(drc-firehose-clouds)


add_executable(drc-triangulation triangulation.cpp)
target_link_libraries(drc-triangulation boost_system)
pods_use_pkg_config_packages(drc-triangulation lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES}  )
pods_install_executables(drc-triangulation)


add_executable(drc-triangulation-smoothing triangulation_smoothing.cpp)
target_link_libraries(drc-triangulation boost_system)
pods_use_pkg_config_packages(drc-triangulation-smoothing lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp rgbd_primitives
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES}  )
pods_install_executables(drc-triangulation-smoothing)




add_executable(drc-send-tracker-command send-tracker-command.cpp)
target_link_libraries(drc-triangulation boost_system)
pods_use_pkg_config_packages(drc-send-tracker-command lcm)
pods_install_executables(drc-send-tracker-command)
