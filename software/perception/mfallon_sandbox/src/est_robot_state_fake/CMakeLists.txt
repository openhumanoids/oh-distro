add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map_velodyne)

add_executable(drc-est-robot-state-fake est-robot-state-fake.cpp  )
#target_link_libraries (drc-est-robot-state-fake ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-est-robot-state-fake  
    lcmtypes_velodyne
    path-util
    bot2-param-client
    bot2-frames
        lcmtypes_kinect lcmtypes_visualization
lcmtypes_drc_lcmtypes
       ${PCL_LIBRARIES})


pods_install_executables(drc-est-robot-state-fake )
