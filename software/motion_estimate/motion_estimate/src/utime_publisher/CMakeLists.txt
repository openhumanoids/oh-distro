add_definitions(-ggdb3 -std=c++0x)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(motion_estimate)

set(REQUIRED_PACKAGES bot2-core lcm drc-utils)

add_executable(drc-utime-publisher utime_publisher.cpp)
pods_use_pkg_config_packages(drc-utime-publisher ${REQUIRED_PACKAGES})
pods_install_executables(drc-utime-publisher)
