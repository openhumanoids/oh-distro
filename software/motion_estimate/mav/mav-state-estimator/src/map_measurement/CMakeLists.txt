#todo: should really name all this

file(GLOB hpp_files *.hpp)
# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est)

# make a state estimator library 
add_library(map-measurement-func SHARED map_measurement_function.cpp)


set(REQUIRED_LIBS  
    mav-state-est 
    gpf-laser-lib
    )
pods_use_pkg_config_packages(map-measurement-func ${REQUIRED_LIBS})
pods_install_libraries(map-measurement-func)
pods_install_pkg_config_file(map-measurement-func
    CFLAGS
    LIBS -lmap-measurement-func
    REQUIRES 	${REQUIRED_LIBS}
    VERSION 0.0.1)

# create an executable, and make it public
add_executable(gen-map-measurement-func gen-map-measurement-func.cpp)
pods_use_pkg_config_packages(gen-map-measurement-func 
    map-measurement-func
)
pods_install_executables(gen-map-measurement-func)

# create an executable, and make it public
add_executable(map-meas-server map-meas-server.cpp)
pods_use_pkg_config_packages(map-meas-server 
    map-measurement-func
)
pods_install_executables(map-meas-server)
