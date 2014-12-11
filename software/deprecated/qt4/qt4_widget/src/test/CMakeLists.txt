# Create an executable program qt4-widget-opengl-test
add_executable(qt4-widget-opengl-test qt4_widget_opengl_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(qt4-widget-opengl-test qt4_widget)

# make executable public
pods_install_executables(qt4-widget-opengl-test)
