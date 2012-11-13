cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(network_bridge)

add_executable(drc-network-bridge network-bridge.cpp   )
target_link_libraries (drc-network-bridge boost_system boost_thread)
pods_use_pkg_config_packages(drc-network-bridge lcm lcmtypes_bot2-core 
	    lcmtypes_drc_lcmtypes eigen3)
pods_install_executables(drc-network-bridge )
