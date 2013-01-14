find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

qt_wrap_cpp(qt4_widget_gfe MOC_SRCS 
            qt4_widget_gfe_control.h
            qt4_widget_gfe_object.h)

# Create a shared library libqt4_widget_gfe.so with a single source file
add_library(qt4_widget_gfe SHARED
            qt4_widget_gfe_control.cc
            qt4_widget_gfe_object.cc
            ${MOC_SRCS})

# make the header public
# install it to include/qt4_widget_gfe
pods_install_headers(qt4_widget_gfe_control.h 
                      qt4_widget_gfe_object.h
                      DESTINATION qt4)

# make the library public
pods_install_libraries(qt4_widget_gfe)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES opengl_object_gfe kinematics_model_gfe state_gfe qt4_widget opengl_scene opengl_object opengl_camera QtCore QtGui QtOpenGL glu gl)
pods_use_pkg_config_packages(qt4_widget_gfe ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(qt4_widget_gfe
    CFLAGS
    LIBS -lqt4_widget_gfe
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
