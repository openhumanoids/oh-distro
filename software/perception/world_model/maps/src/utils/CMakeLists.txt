add_definitions(    -ggdb3     -std=gnu99    )
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(heightmap_lcmgl)

add_executable(drc-gradmap-send gradmap_send.cpp  )
pods_use_pkg_config_packages(drc-gradmap-send
    path-util
    bot2-lcmgl-client 
    bot2-param-client
    bot2-frames
    drc-lcmtypes
    eigen3
       ${PCL_LIBRARIES})
pods_install_executables(drc-gradmap-send)
