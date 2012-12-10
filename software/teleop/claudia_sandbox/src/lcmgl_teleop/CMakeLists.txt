add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(drive_map)

add_executable(drc-lcmgl-teleop lcmgl_teleop.cpp  )
pods_use_pkg_config_packages(drc-lcmgl-teleop  
    bot2-lcmgl-client lcmtypes_vicon-drc)


pods_install_executables(drc-lcmgl-teleop )
