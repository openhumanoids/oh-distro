find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

qt_wrap_cpp(opengl_widget MOC_SRCS opengl_widget.h)

# Create a shared library libopengl_widget.so with a single source file
add_library(opengl_widget SHARED
            opengl_widget.cc
            ${MOC_SRCS})

# make the header public
# install it to include/opengl_widget
pods_install_headers(opengl_widget.h 
                      DESTINATION opengl_widget)

# make the library public
pods_install_libraries(opengl_widget)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES opengl_scene opengl_object opengl_camera QtCore QtGui QtOpenGL glu gl)
pods_use_pkg_config_packages(opengl_widget ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(opengl_widget
    CFLAGS
    LIBS -lopengl_widget
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
