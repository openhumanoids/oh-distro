################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(camera_params)

add_library(camera_params SHARED camera_params.cpp)

pods_use_pkg_config_packages(camera_params lcm lcmtypes_bot2-core)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(camera_params PROPERTIES SOVERSION 1)
pods_install_libraries(camera_params)
pods_install_headers(camera_params.hpp DESTINATION camera_params )

target_link_libraries(camera_params
    z)
pods_install_pkg_config_file(camera_params
    LIBS -lcamera_params -lz
    REQUIRES image-utils lcmtypes_bot2-core
    VERSION 0.0.1)

