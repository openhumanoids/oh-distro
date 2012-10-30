cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_segment)

add_executable(drc-network-bridge network-bridge.cpp   )
target_link_libraries (drc-network-bridge boost_system)
pods_use_pkg_config_packages(drc-network-bridge lcm lcmtypes_bot2-core)
#        pointcloud_lcm         pointcloud_vis         pointcloud_math
#	lcmtypes_drc_lcmtypes  lcmtypes_visualization )
pods_install_executables(drc-network-bridge )
