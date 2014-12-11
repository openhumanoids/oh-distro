# Create a shared library libinterface.so with a single source file
add_library(interface SHARED
            interface_handler.cc)

# make the header public
# install it to include/interface
pods_install_headers(interface_handler.h DESTINATION interface)

# make the library public
pods_install_libraries(interface)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES collision_detector collision_object state glu gl)
pods_use_pkg_config_packages(interface ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(interface
    CFLAGS
    LIBS -linterface
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
