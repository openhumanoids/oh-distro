################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(voestimator)

add_library(voestimator SHARED voestimator.cpp)

pods_use_pkg_config_packages(voestimator eigen3 pointcloud_vis bot2-frames)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(voestimator PROPERTIES SOVERSION 1)
pods_install_libraries(voestimator)
pods_install_headers(voestimator.hpp DESTINATION drcvision)
pods_install_pkg_config_file(voestimator
    LIBS -lvoestimator 
    REQUIRES bot2-frames bot2-frames_cpp
    VERSION 0.0.1)

