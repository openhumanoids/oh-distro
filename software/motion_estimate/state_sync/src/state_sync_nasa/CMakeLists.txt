set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync_val)


add_executable(drc-state-sync-val state_sync_nasa.cpp  )
pods_use_pkg_config_packages(drc-state-sync-val eigen3
  model-client bot2-param-client bot2-frames drake-rbm
  kalman_filter_tools backlash_filter_tools torque_adjustment_tools
  lcmtypes_drc_lcmtypes lcmtypes_bot2-core)
pods_install_executables(drc-state-sync-val)
