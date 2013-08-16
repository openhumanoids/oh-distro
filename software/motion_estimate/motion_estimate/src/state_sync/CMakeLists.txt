set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(state_sync)


add_executable(drc-state-sync state_sync.cpp  )
pods_use_pkg_config_packages(drc-state-sync eigen3
  lcmtypes_drc_lcmtypes)
pods_install_executables(drc-state-sync )

#add_executable(drc-robot-utime-creator robot_utime_creator.cpp  )
#pods_use_pkg_config_packages(drc-robot-utime-creator
#  lcmtypes_drc_lcmtypes)
#pods_install_executables(drc-robot-utime-creator )


# not required any more
#add_executable(drc-state-converter state_converter.cpp  )
#pods_use_pkg_config_packages(drc-state-converter
#  lcmtypes_drc_lcmtypes)
#
#pods_install_executables(drc-state-converter )
