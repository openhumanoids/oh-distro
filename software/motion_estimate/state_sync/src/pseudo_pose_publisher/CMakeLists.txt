add_definitions(-ggdb3 -std=c++0x)

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(motion_estimate)

set(REQUIRED_PACKAGES bot2-core lcm drc-utils)
set(EXE_NAME pseudo-pose-publisher)

add_executable(${EXE_NAME} pseudo_pose_publisher.cpp)
pods_use_pkg_config_packages(${EXE_NAME} ${REQUIRED_PACKAGES})
pods_install_executables(${EXE_NAME})
