# Create a shared library libcollision_object.so with a single source file
add_library(collision_object_container SHARED
              collision_object_container_coordinate_axis.cc)

# make the header public
# install it to include/collision_object
pods_install_headers(collision_object_container_coordinate_axis.h
                      DESTINATION collision)

# make the library public
pods_install_libraries(collision_object_container)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES collision_object collision_detector kinematics_model path-util urdf bullet eigen3)
pods_use_pkg_config_packages(collision_object_container ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(collision_object_container
    CFLAGS
    LIBS -lcollision_object_container
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
