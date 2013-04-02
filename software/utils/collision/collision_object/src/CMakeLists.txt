# Create a shared library libcollision_object.so with a single source file
add_library(collision_object SHARED
              collision_object.cc
              collision_object_box.cc
              collision_object_plane.cc
              collision_object_cone.cc
              collision_object_cylinder.cc
              collision_object_torus.cc
              collision_object_sphere.cc
              collision_object_convex_hull.cc
              collision_object_urdf.cc
              collision_object_point_cloud.cc)

# make the header public
# install it to include/collision_object
pods_install_headers(collision_object.h
                      collision_object_box.h
                      collision_object_plane.h
                      collision_object_cone.h
                      collision_object_cylinder.h
                      collision_object_torus.h
                      collision_object_sphere.h
                      collision_object_convex_hull.h
                      collision_object_urdf.h
                      collision_object_point_cloud.h
                      DESTINATION collision)

# make the library public
pods_install_libraries(collision_object)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES kinematics_model path-util urdf bullet eigen3)
pods_use_pkg_config_packages(collision_object ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(collision_object
    CFLAGS
    LIBS -lcollision_object
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
