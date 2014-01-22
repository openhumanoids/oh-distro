#build the state estimation library
add_subdirectory(mav_state_est)
add_subdirectory(gpf)

# create the fixie state estimation executable, and make it public
add_executable(mav-state-estimator mav_state_estimator.cpp)
pods_use_pkg_config_packages(mav-state-estimator 
    mav-state-est
    gpf-laser-lib)
pods_install_executables(mav-state-estimator)

#create an executable for a simple vicon state estimation
add_executable(fixie-vicon-simple-state-estimator vicon_simple_state_estimator.cpp)
pods_use_pkg_config_packages(fixie-vicon-simple-state-estimator 
	bot2-core
	bot2-param-client
	bot2-frames
	eigen-utils
	lcmtypes_microstrain_comm
    path-util
    lcmtypes_vicon
    lcmtypes_mav-lcmtypes
)
pods_install_executables(fixie-vicon-simple-state-estimator)


add_subdirectory(noise_id)
add_subdirectory(map_measurement)
add_subdirectory(renderers)
add_subdirectory(testers)
