cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(estimate)



################
add_executable(se-comp main.cpp )
pods_use_pkg_config_packages(se-comp bot2-frames_cpp
    pointcloud_vis  )
target_link_libraries (se-comp lcm pointcloud_math  )
pods_install_executables(se-comp )
