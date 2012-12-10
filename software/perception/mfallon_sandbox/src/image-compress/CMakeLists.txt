add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(image-compress)

add_executable(drc-image-compress image-compress.cpp  )
#target_link_libraries(drc-image-compress jpeg z  m)
pods_use_pkg_config_packages(drc-image-compress  jpeg-utils
    lcmtypes_bot2-core opencv)


pods_install_executables(drc-image-compress )
