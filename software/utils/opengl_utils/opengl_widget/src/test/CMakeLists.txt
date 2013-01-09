# Create an executable program opengl-widget-test
add_executable(opengl-widget-test opengl_widget_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-widget-test opengl_widget)

# make executable public
pods_install_executables(opengl-widget-test)
