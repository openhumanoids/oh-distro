add_definitions(    -ggdb3     -std=gnu99    )
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(heightmap_lcmgl)

add_executable(drc-heightmap-lcmgl heightmap_lcmgl.cpp  )
pods_use_pkg_config_packages(drc-heightmap-lcmgl  
    path-util
    bot2-lcmgl-client 
    bot2-param-client
    bot2-frames
    lcmtypes_visualization
    drc-lcmtypes
    eigen3
       ${PCL_LIBRARIES})
pods_install_executables(drc-heightmap-lcmgl )


add_executable(drc-octomap-trigger drc-octomap-trigger.cpp  )
pods_use_pkg_config_packages(drc-octomap-trigger
    path-util
    bot2-lcmgl-client 
    bot2-param-client
    bot2-frames
    drc-lcmtypes
    eigen3
       ${PCL_LIBRARIES})
pods_install_executables(drc-octomap-trigger)


#add_executable(drc-gradmap-send gradmap_send.cpp  )
#pods_use_pkg_config_packages(drc-gradmap-send
#    path-util
#    bot2-lcmgl-client 
#    bot2-param-client
#    bot2-frames
#    drc-lcmtypes
#    eigen3
#       ${PCL_LIBRARIES})
#pods_install_executables(drc-gradmap-send)
