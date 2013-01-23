# Create a shared library libhello.so with a single source file
add_library(hello SHARED
    hello.c)

# make the header public
# install it to include/hello
pods_install_headers(hello.h DESTINATION hello)

# make the library public
pods_install_libraries(hello)

# uncomment these lines to link against another library via pkg-config
#set(REQUIRED_PACKAGES pkg_a pkg_b)
#pods_use_pkg_config_packages(hello ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(hello
    CFLAGS
    LIBS -lhello
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
