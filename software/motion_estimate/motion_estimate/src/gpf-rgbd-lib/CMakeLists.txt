file(GLOB hpp_files *.hpp)

# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est/gpf-rgbd-lib)

# make a state estimator library 
add_library(gpf-rgbd-lib SHARED 
                rgbd_gpf_lib.cpp 
                gpf.cpp 
                RgbdLikelihoodInterface.cpp
                rbis_gpf_update.cpp)


set(REQUIRED_LIBS mav-state-est 
    bot2-core
	bot2-param-client
	bot2-frames
	eigen-utils
	octomap-util
	lcmtypes_microstrain_comm
    path-util
    lcmtypes_vicon
    lcmtypes_mav_estimator
    lcmtypes_kinect
    )
#	laser-util
#	laser-sim3d


pods_use_pkg_config_packages(gpf-rgbd-lib ${REQUIRED_LIBS})
pods_install_libraries(gpf-rgbd-lib)
pods_install_pkg_config_file(gpf-rgbd-lib
    CFLAGS
    LIBS -lgpf-rgbd-lib
    REQUIRES 	${REQUIRED_LIBS}
    VERSION 0.0.1)




#add_executable(gpf-rgbd rgbd_gpf.cpp)
#pods_use_pkg_config_packages(gpf-rgbd gpf-rgbd-lib)
#pods_install_executables(gpf-rgbd)


#create simple program to create an octomap on the z-plane
#add_executable(gpf-gen-z-plane-octomap gen-z-plane-octomap.cpp)
#pods_use_pkg_config_packages(gpf-gen-z-plane-octomap octomap-util)
#pods_install_executables(gpf-gen-z-plane-octomap)
