# Create a shared library libcollision_object_gfe.so with a single source file
add_library(collision_object_gfe SHARED
            collision_object_gfe.cc)

# make the header public
# install it to include/collision_object_gfe
pods_install_headers(collision_object_gfe.h DESTINATION collision)

# make the library public
pods_install_libraries(collision_object_gfe)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES kinematics_model_gfe collision_detector collision_object kinematics_model path-util bullet eigen3)
pods_use_pkg_config_packages(collision_object_gfe ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(collision_object_gfe
    CFLAGS
    LIBS -lcollision_object_gfe
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
