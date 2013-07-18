add_definitions( -ggdb3 -std=gnu99 )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)

add_executable(drc-motion-estimate-imu-ypr-only imu_to_pose.cpp  )
pods_use_pkg_config_packages(drc-motion-estimate-imu-ypr-only
	lcmtypes_drc_lcmtypes)

pods_install_executables(drc-motion-estimate-imu-ypr-only )
