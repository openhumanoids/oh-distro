add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(drive_map)

add_executable(drc-lcmgl-example lcmgl_example.cpp  )
pods_use_pkg_config_packages(drc-lcmgl-example  
    bot2-lcmgl-client opencv)


pods_install_executables(drc-lcmgl-example )
