# Create a shared library libmav-renderers.so with a single source file
add_library(mav-state-est-renderers SHARED
    map_measurement_renderer.cpp
    mav_state_est_renderer.cpp)

set(REQUIRED_LIBS 
	mav-state-est
	bot2-vis 
	eigen-utils
	map-measurement-func)

pods_use_pkg_config_packages(mav-state-est-renderers ${REQUIRED_LIBS})

# make the header public
# install it to include/mav-renderers
pods_install_headers(mav_state_est_renderers.h DESTINATION mav_state_est)

# make the library public
pods_install_libraries(mav-state-est-renderers)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(mav-state-est-renderers
    CFLAGS
    LIBS -lmav-state-est-renderers
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
