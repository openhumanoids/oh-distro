set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync)


add_executable(nasa-state-sync state_sync_nasa.cpp  )
pods_use_pkg_config_packages(nasa-state-sync eigen3
  model-client bot2-param-client bot2-frames
  kalman_filter_tools backlash_filter_tools torque_adjustment_tools
  lcmtypes_drc_lcmtypes)
pods_install_executables(nasa-state-sync )
