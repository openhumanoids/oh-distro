# Create a shared library libopengl_object_authoring.so with a single source file
add_library(opengl_object_authoring SHARED
            opengl_object_affordance.cc
            opengl_object_affordance_collection.cc)

# make the header public
# install it to include/opengl_object_authoring
pods_install_headers(opengl_object_affordance.h
                      opengl_object_affordance_collection.h 
                      DESTINATION authoring)

# make the library public
pods_install_libraries(opengl_object_authoring)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES affordance opengl_object visualization-utils)
pods_use_pkg_config_packages(opengl_object_authoring ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(opengl_object_authoring
    CFLAGS
    LIBS -lopengl_object_authoring
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
