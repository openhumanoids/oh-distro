set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6 pcl_filters-1.6)

# spoof a CANDIDATE_GRASP or a the input to the planner:
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)
add_executable (affordance_to_grasp_plan affordance_to_grasp_plan.cpp )
target_link_libraries(affordance_to_grasp_plan boost_system)
pods_use_pkg_config_packages(affordance_to_grasp_plan 
model-client ${REQUIRED_PACKAGES} pointcloud_vis
   affordance-utils lcm)
pods_install_executables(affordance_to_grasp_plan)


# spoof an affordance near the shoulder
add_executable (affordance_init_relative affordance_init_relative.cpp )
target_link_libraries(affordance_init_relative boost_system)
pods_use_pkg_config_packages(affordance_init_relative 
model-client ${REQUIRED_PACKAGES} pointcloud_vis
  bot2-param-client 
  bot2-frames  bot2-frames_cpp
   affordance-utils lcm)
pods_install_executables(affordance_init_relative)

# spoof ee goals near the shoulder
add_executable (ee-test ee_test.cpp )
target_link_libraries(ee-test boost_system)
pods_use_pkg_config_packages(ee-test
model-client ${REQUIRED_PACKAGES} pointcloud_vis
  bot2-param-client 
  bot2-frames  bot2-frames_cpp
   affordance-utils lcm)
pods_install_executables(ee-test)


##############################################
add_executable (ee-driver ee_driver.cpp)
target_link_libraries (ee-driver curses boost_system   lcm)
pods_use_pkg_config_packages(ee-driver lcm model-client
   lcmtypes_drc_lcmtypes  bot2-core glib-2.0
   pointcloud_math
   eigen3 ${REQUIRED_PACKAGES})
pods_install_executables(ee-driver)



##############################################
add_executable (affordance_debris_util affordance_debris_util.cpp)
target_link_libraries (affordance_debris_util curses boost_system   lcm)
pods_use_pkg_config_packages(affordance_debris_util lcm model-client
   lcmtypes_drc_lcmtypes  bot2-core glib-2.0
   pointcloud_math
   eigen3 ${REQUIRED_PACKAGES})
pods_install_executables(affordance_debris_util)



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




###################################
# Convert Mini to regualr affordance on the robot
add_executable (affordance_mini_rx affordance_mini_rx.cpp )
target_link_libraries(affordance_mini_rx boost_system)
pods_use_pkg_config_packages(affordance_mini_rx  lcm  affordance-utils
  pointcloud_vis
  ${PCL_LIBRARIES})
pods_install_executables(affordance_mini_rx)
