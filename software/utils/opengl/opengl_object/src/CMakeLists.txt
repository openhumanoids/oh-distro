# Create a shared library libopengl_object.so with a single source file
add_library(opengl_object SHARED
            opengl_object.cc
            opengl_object_plane.cc
            opengl_object_box.cc
            opengl_object_sphere.cc
            opengl_object_cylinder.cc
            opengl_object_torus.cc
            opengl_object_dae.cc
            opengl_object_coordinate_axis.cc
            opengl_object_collision_detector.cc)

# make the header public
# install it to include/opengl_object
pods_install_headers(opengl_object.h
                      opengl_object_box.h
                      opengl_object_plane.h
                      opengl_object_sphere.h
                      opengl_object_cylinder.h 
                      opengl_object_torus.h 
                      opengl_object_dae.h
                      opengl_object_coordinate_axis.h
                      opengl_object_collision_detector.h
                      DESTINATION opengl)

# make the library public
pods_install_libraries(opengl_object)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES collision_detector collision_object bullet eigen3 orocos-kdl kdl-parser path-util libxml-2.0 glu gl)
pods_use_pkg_config_packages(opengl_object ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(opengl_object
    CFLAGS
    LIBS -lopengl_object
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
