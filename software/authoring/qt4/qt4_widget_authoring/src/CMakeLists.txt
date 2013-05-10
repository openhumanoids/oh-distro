find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

qt_wrap_cpp(qt4_widget_authoring MOC_SRCS
              qt4_widget_double_slider.h
              qt4_widget_constraint_editor.h
              qt4_widget_constraint_task_space_region_editor.h
              qt4_widget_opengl_authoring.h
              qt4_widget_authoring.h
              qt4_widget_authoring_lcm_interface.h)

# Create a shared library libqt4_widget_authoring.so with a single source file
add_library(qt4_widget_authoring SHARED
            qt4_widget_double_slider.cc
            qt4_widget_constraint_editor.cc
            qt4_widget_constraint_task_space_region_editor.cc
            qt4_widget_opengl_authoring.cc
            qt4_widget_authoring.cc
            qt4_widget_authoring_lcm_interface.cc
            ${MOC_SRCS})

# make the header public
# install it to include/qt4_widget_authoring
pods_install_headers(qt4_widget_double_slider.h
                      qt4_widget_constraint_editor.h
                      qt4_widget_constraint_task_space_region_editor.h
                      qt4_widget_opengl_authoring.h 
                      qt4_widget_authoring.h 
                      qt4_widget_authoring_lcm_interface.h
                      DESTINATION authoring)

# make the library public
pods_install_libraries(qt4_widget_authoring)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES opengl_object_authoring constraint_authoring opengl_object_gfe state_gfe affordance qt4_widget opengl_object lcm)
pods_use_pkg_config_packages(qt4_widget_authoring ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(qt4_widget_authoring
    CFLAGS
    LIBS -lqt4_widget_authoring
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
