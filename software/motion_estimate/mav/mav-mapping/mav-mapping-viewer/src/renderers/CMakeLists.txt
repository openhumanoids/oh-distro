add_definitions(-std=gnu99)

# Create a shared library libquad-renderers.so with a single source file
add_library(mav-mapping-renderers SHARED
    renderer_point_cloud_registration.c)

set(REQUIRED_LIBS bot2-core bot2-vis)

pods_use_pkg_config_packages(mav-mapping-renderers ${REQUIRED_LIBS})

# make the header public
# install it to include/mav-mapping-renderers
pods_install_headers(mav_mapping_renderers.h DESTINATION mav_mapping_renderers)

# make the library public
pods_install_libraries(mav-mapping-renderers)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(mav-mapping-renderers
    CFLAGS
    LIBS -lmav-mapping-renderers
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
