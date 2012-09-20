add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(velodyne2pcd)

add_executable(drc-velodyne2pcd velodyne2pcd.cpp  )
pods_use_pkg_config_packages(drc-velodyne2pcd
    lcmtypes_velodyne
    path-util
    bot2-param-client
        velodyne)
pods_install_executables(drc-velodyne2pcd )

add_executable(drc-velodyne2pcd-continous velodyne2pcd_continous.cpp  )
pods_use_pkg_config_packages(drc-velodyne2pcd-continous
    lcmtypes_velodyne
    path-util
    bot2-param-client
    bot2-frames
        velodyne)
pods_install_executables(drc-velodyne2pcd-continous )
