add_definitions( -ggdb3 -std=gnu99 )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)

add_executable(se-blacken-image blacken_image.cpp  )
pods_use_pkg_config_packages(se-blacken-image
  image_io_utils opencv
	lcmtypes_drc_lcmtypes eigen3)

pods_install_executables(se-blacken-image )
