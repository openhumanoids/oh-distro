################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(voconfig)

add_library(voconfig SHARED voconfig.cpp)

pods_use_pkg_config_packages(voconfig bot2-param-client eigen3)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(voconfig PROPERTIES SOVERSION 1)
pods_install_libraries(voconfig)
pods_install_headers(voconfig.hpp DESTINATION drcvision)
pods_install_pkg_config_file(voconfig
    LIBS -lvoconfig 
    REQUIRES bot2-param-client
    VERSION 0.0.1)

