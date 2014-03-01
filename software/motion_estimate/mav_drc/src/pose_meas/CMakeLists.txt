file(GLOB hpp_files *.hpp)

# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est)


file(GLOB cpp_files *.cpp)

# make a state estimator library 
add_library(mav-est-pose-init SHARED ${cpp_files})

set(REQUIRED_LIBS bot2-core
	bot2-param-client
	bot2-frames
	eigen-utils
    path-util
    lcmtypes_mav-lcmtypes
    )
pods_use_pkg_config_packages(mav-est-pose-init ${REQUIRED_LIBS})
pods_install_libraries(mav-est-pose-init)
pods_install_pkg_config_file(mav-est-pose-init
    CFLAGS
    LIBS -lmav-est-pose-init
    REQUIRES 	${REQUIRED_LIBS}
    VERSION 0.0.1)
