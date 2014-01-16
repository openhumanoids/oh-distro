cmake_minimum_required (VERSION 2.6)

add_executable(drc-tags-demo apriltags_demo.cpp)
pods_use_pkg_config_packages(drc-tags-demo apriltags opencv)
pods_install_executables(drc-tags-demo)
