cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)


set(PCL_LIBRARIES  pcl_io-1.6)
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(drc-leg-odometry leg_odometry.cpp foot_contact.cpp )
pods_use_pkg_config_packages(drc-leg-odometry
    path-util model-client   bot2-param-client
        pointcloud_lcm        pointcloud_vis        pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES} ${REQUIRED_PACKAGES})
pods_install_executables(drc-leg-odometry )