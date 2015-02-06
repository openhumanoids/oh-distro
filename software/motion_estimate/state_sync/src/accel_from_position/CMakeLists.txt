#add_definitions(
#    -ggdb3 
#    -std=gnu99
#    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)

set(PCL_LIBRARIES  pcl_io-1.7)
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(drc-accel-from-position accel_from_position.cpp  )
target_link_libraries(drc-accel-from-position boost_system)
pods_use_pkg_config_packages(drc-accel-from-position
    path-util model-client   bot2-param-client  bot2-frames bot2-frames_cpp
        pronto_lcm        pronto_vis        pronto_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES} ${REQUIRED_PACKAGES})
pods_install_executables(drc-accel-from-position )
