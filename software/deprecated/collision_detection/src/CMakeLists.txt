# Create a shared library libcollision_detection.so with a single source file
add_library(collision-detection SHARED
              collision.cc
              collision_detector.cc
              collision_object.cc
              collision_object_box.cc
              collision_object_cone.cc
              collision_object_cylinder.cc
              collision_object_sphere.cc
              collision_object_convex_hull.cc
              collision_object_point_cloud.cc
              collision_object_gfe.cc)

# make the header public
# install it to include/collision_detection
pods_install_headers(collision.h
                      collision_detector.h
                      collision_object.h
                      collision_object_box.h
                      collision_object_cone.h
                      collision_object_cylinder.h
                      collision_object_sphere.h
                      collision_object_convex_hull.h
                      collision_object_point_cloud.h
                      collision_object_gfe.h
                      DESTINATION collision_detection)

# make the library public
pods_install_libraries(collision-detection)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES kinematics_model path-util bullet eigen3)
pods_use_pkg_config_packages(collision-detection ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(collision_detection
    CFLAGS
    LIBS -lcollision-detection
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
