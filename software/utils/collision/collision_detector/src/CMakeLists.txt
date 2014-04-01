# Create a shared library libcollision_detector.so with a single source file
add_library(collision_detector SHARED
              collision.cc
              collision_detector.cc)

# make the header public
# install it to include/collision_detector
pods_install_headers(collision.h
                      collision_detector.h
                      DESTINATION collision)

# make the library public
pods_install_libraries(collision_detector)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES collision_object path-util bullet eigen3)
pods_use_pkg_config_packages(collision_detector ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(collision_detector
    CFLAGS
    LIBS -lcollision_detector
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
