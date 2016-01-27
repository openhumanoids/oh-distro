set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync)


add_executable(drc-state-sync-atlas state_sync.cpp  )
pods_use_pkg_config_packages(drc-state-sync-atlas eigen3
  model-client bot2-param-client bot2-frames
  kalman_filter_tools backlash_filter_tools torque_adjustment_tools
  lcmtypes_drc_lcmtypes)
pods_install_executables(drc-state-sync-atlas )


add_executable(drc-update-offsets update_offsets.cpp  )
pods_use_pkg_config_packages(drc-update-offsets eigen3
  lcmtypes_drc_lcmtypes atlas bot2-param-client bot2-frames)
pods_install_executables(drc-update-offsets )
