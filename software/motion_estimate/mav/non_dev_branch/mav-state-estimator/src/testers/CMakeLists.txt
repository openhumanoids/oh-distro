add_executable(test_laser_project_with_motion test_laser_project_with_motion.cpp)
pods_use_pkg_config_packages(test_laser_project_with_motion 
	bot2-core
	bot2-param-client
	bot2-frames
	eigen-utils
	lcmtypes_microstrain_comm
    path-util
    laser-util
    lcmtypes_vicon
)
