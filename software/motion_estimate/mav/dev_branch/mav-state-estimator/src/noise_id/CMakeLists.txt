file(GLOB hpp_files *.hpp)

# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est)

file(GLOB cpp_files *.cpp)

# make a state estimator library 
add_library(mav-state-est-noise-id SHARED ${cpp_files})

set(REQUIRED_LIBS
	mav-state-est
    )
pods_use_pkg_config_packages(mav-state-est-noise-id ${REQUIRED_LIBS})
pods_install_libraries(mav-state-est-noise-id)
pods_install_pkg_config_file(mav-state-est-noise-id
    CFLAGS
    LIBS -lmav-state-est-noise-id
    REQUIRES 	${REQUIRED_LIBS}
    VERSION 0.0.1)
    
    
    
add_executable(mav-noise-id-rollforward roll_forward.cpp)
pods_use_pkg_config_packages(mav-noise-id-rollforward lcm mav-state-est-noise-id)
pods_install_executables(mav-noise-id-rollforward)
    