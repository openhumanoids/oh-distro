# Create a shared library libcollision_detection.so with a single source file
add_library(collision-detection SHARED
              collision_detector.cc
              collision_object.cc
              collision_object_box.cc)

# make the header public
# install it to include/collision_detection
pods_install_headers(collision_detector.h 
                      collision_object.h
                      collision_object_box.h
                      DESTINATION collision_detection)

# make the library public
pods_install_libraries(collision-detection)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES bullet eigen3)
pods_use_pkg_config_packages(collision-detection ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(collision-detection
    CFLAGS
    LIBS -lcollision-detection
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
