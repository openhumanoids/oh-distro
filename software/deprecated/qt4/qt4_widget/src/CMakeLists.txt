find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

qt_wrap_cpp(qt4_widget MOC_SRCS qt4_widget_opengl.h)

# Create a shared library libqt4_widget.so with a single source file
add_library(qt4_widget SHARED
            qt4_widget_opengl.cc
            ${MOC_SRCS})

# make the header public
# install it to include/qt4_widget
pods_install_headers(qt4_widget_opengl.h 
                      DESTINATION qt4)

# make the library public
pods_install_libraries(qt4_widget)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES interface collision_detector collision_object opengl_scene opengl_object opengl_camera bullet QtCore QtGui QtOpenGL glu gl)
pods_use_pkg_config_packages(qt4_widget ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(qt4_widget
    CFLAGS
    LIBS -lqt4_widget
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
