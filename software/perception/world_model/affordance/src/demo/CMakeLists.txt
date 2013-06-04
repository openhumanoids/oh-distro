set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)


############ Demo Apps: ###########################
# send one poly:
add_executable (affordance_points_demo affordance_points_demo.cpp )
pods_use_pkg_config_packages(affordance_points_demo  lcm)
pods_install_executables(affordance_points_demo)

### needed to read meshes:
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

# send points and polys from file:
add_executable (affordance_points_demo_plus affordance_points_demo_plus.cpp )
target_link_libraries(affordance_points_demo_plus boost_system)
pods_use_pkg_config_packages(affordance_points_demo_plus  lcm 
  pointcloud_vis
  ${PCL_LIBRARIES})
pods_install_executables(affordance_points_demo_plus)

# receive and render the polygons and points
add_executable (affordance_points_demo_plus_receive affordance_points_demo_plus_receive.cpp )
target_link_libraries(affordance_points_demo_plus_receive boost_system)
pods_use_pkg_config_packages(affordance_points_demo_plus_receive  lcm 
  pointcloud_vis affordance-utils
  ${PCL_LIBRARIES})
pods_install_executables(affordance_points_demo_plus_receive)

###################################
# Send the drill model as an affordance_fit
add_executable (affordance_init affordance_init.cpp )
target_link_libraries(affordance_init boost_system)
pods_use_pkg_config_packages(affordance_init  lcm  affordance-utils
  pointcloud_vis
  ${PCL_LIBRARIES})
pods_install_executables(affordance_init)

###################################
# Send the drill model as an affordance_fit
add_executable (ply2pcd ply2pcd.cpp )
target_link_libraries(ply2pcd boost_system)
pods_use_pkg_config_packages(ply2pcd  lcm  affordance-utils
  pointcloud_vis
  ${PCL_LIBRARIES})
pods_install_executables(ply2pcd)
