################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(image_io_utils)

add_library(image_io_utils SHARED image_io_utils.cpp)

pods_use_pkg_config_packages(image_io_utils lcm lcmtypes_bot2-core)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(image_io_utils PROPERTIES SOVERSION 1)
pods_install_libraries(image_io_utils)
pods_install_headers(image_io_utils.hpp DESTINATION image_io_utils )

target_link_libraries(image_io_utils
    z)
pods_install_pkg_config_file(image_io_utils
    LIBS -limage_io_utils -lz
    REQUIRES image-utils lcmtypes_bot2-core
    VERSION 0.0.1)

