# Create a shared library libstate.so with a single source file
add_library(state SHARED
            state.cc)

# make the header public
# install it to include/state
pods_install_headers(state.h DESTINATION state)

# make the library public
pods_install_libraries(state)

# uncomment these lines to link against another library via pkg-config
#set(REQUIRED_PACKAGES pkg_a pkg_b)
#pods_use_pkg_config_packages(state ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(state
    CFLAGS
    LIBS -lstate
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
