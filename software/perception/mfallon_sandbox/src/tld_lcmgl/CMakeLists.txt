add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(drive_map)

add_executable(drc-tld_lcmgl tld_lcmgl.cpp  )
pods_use_pkg_config_packages(drc-tld_lcmgl  
  lcmtypes_tld-tracker
    bot2-lcmgl-client opencv eigen3 pointcloud_math
    lcmtypes_drc_lcmtypes
  bot2-frames bot2-frames_cpp drc-utils)


pods_install_executables(drc-tld_lcmgl )
