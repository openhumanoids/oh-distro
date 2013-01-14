# Create a shared library libstate_gfe.so with a single source file
add_library(state_gfe SHARED
            state_gfe_joint.cc
            state_gfe.cc)

# make the header public
# install it to include/state_gfe
pods_install_headers(state_gfe_joint.h
                      state_gfe.h 
                      DESTINATION state)

# make the library public
pods_install_libraries(state_gfe)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES kdl-parser path-util urdf)
pods_use_pkg_config_packages(state_gfe ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(state_gfe
    CFLAGS
    LIBS -lstate_gfe
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
