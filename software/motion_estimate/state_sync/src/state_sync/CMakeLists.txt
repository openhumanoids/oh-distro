set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync)


add_library(state_sync  SHARED state_sync.cpp)
pods_use_pkg_config_packages(state_sync  eigen3
  model-client bot2-param-client bot2-frames
  kalman_filter_tools backlash_filter_tools torque_adjustment_tools
  lcmtypes_drc_lcmtypes)
set_target_properties(state_sync PROPERTIES SOVERSION 1)
pods_install_libraries(state_sync)
pods_install_headers( state_sync.hpp 
               DESTINATION state_sync)
pods_install_pkg_config_file(state_sync
  LIBS -lstate_sync
  VERSION 0.0.1)

#  LIBS -lleg_estimate  -lpcl_io  -lfilter_tools -lkalman_filter_tools
#   REQUIRES pcl_io-1.6
add_executable(drc-state-sync state_sync_app.cpp  )
pods_use_pkg_config_packages(drc-state-sync state_sync lcm)
pods_install_executables(drc-state-sync )




add_executable(drc-arm-sensor-dump arm_sensor_dump.cpp  )
pods_use_pkg_config_packages(drc-arm-sensor-dump eigen3
  lcmtypes_drc_lcmtypes atlas)
pods_install_executables(drc-arm-sensor-dump )





add_executable(drc-update-offsets update_offsets.cpp  )
pods_use_pkg_config_packages(drc-update-offsets eigen3
  lcmtypes_drc_lcmtypes atlas bot2-param-client bot2-frames)
pods_install_executables(drc-update-offsets )
