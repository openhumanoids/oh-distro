set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync)


add_executable(drc-state-sync-kuka state-sync-kuka.cpp  )
pods_use_pkg_config_packages(drc-state-sync-kuka eigen3
  model-client bot2-param-client bot2-frames
  lcmtypes_drc_lcmtypes )
#include_directory(boost_system)
target_link_libraries(drc-state-sync-kuka  boost_thread boost_system)

pods_install_executables(drc-state-sync-kuka )

