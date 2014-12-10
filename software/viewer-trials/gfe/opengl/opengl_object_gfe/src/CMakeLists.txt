# Create a shared library libhello.so with a single source file
add_library(opengl_object_gfe SHARED
            opengl_object_gfe.cc
            opengl_object_trajectory_gfe.cc)

# make the header public
# install it to include/hello
pods_install_headers(opengl_object_gfe.h 
                      opengl_object_trajectory_gfe.h
                      DESTINATION opengl)

# make the library public
pods_install_libraries(opengl_object_gfe)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES opengl_object kinematics_model_gfe kinematics_model eigen3 orocos-kdl kdl-parser path-util libxml-2.0 glu gl)
pods_use_pkg_config_packages(opengl_object_gfe ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(opengl_object_gfe
    CFLAGS
    LIBS -lopengl_object_gfe
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
