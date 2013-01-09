# Create a shared library libopengl_scene.so with a single source file
add_library(opengl_scene SHARED
            opengl_scene.cc)

# make the header public
# install it to include/opengl_scene
pods_install_headers(opengl_scene.h DESTINATION opengl_scene)

# make the library public
pods_install_libraries(opengl_scene)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES opengl_camera opengl_object eigen3 glu gl)
pods_use_pkg_config_packages(opengl_scene ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(opengl_scene
    CFLAGS
    LIBS -lopengl_scene
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
