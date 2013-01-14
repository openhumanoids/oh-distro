# Create a shared library libopengl_camera.so with a single source file
add_library(opengl_camera SHARED
            opengl_camera.cc)

# make the header public
# install it to include/opengl_camera
pods_install_headers(opengl_camera.h DESTINATION opengl)

# make the library public
pods_install_libraries(opengl_camera)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES eigen3 glu gl)
pods_use_pkg_config_packages(opengl_camera ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(opengl_camera
    CFLAGS
    LIBS -lopengl_camera
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
