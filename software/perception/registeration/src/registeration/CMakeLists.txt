add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)



add_executable(drc-registeration registeration.cpp  )
#target_link_libraries (drc-registeration-velodyne ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-registeration
    path-util
    bot2-param-client
        pointcloud_lcm        pointcloud_vis        pointcloud_math
         opencv
	lcmtypes_drc_lcmtypes  lcmtypes_visualization)


pods_install_executables(drc-registeration )
