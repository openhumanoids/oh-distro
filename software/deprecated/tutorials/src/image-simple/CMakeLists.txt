add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(image-simple)

add_executable(drc-image-simple image-simple.cpp  )
pods_use_pkg_config_packages(drc-image-simple  jpeg-utils
    lcmtypes_bot2-core opencv)

pods_install_executables(drc-image-simple )

