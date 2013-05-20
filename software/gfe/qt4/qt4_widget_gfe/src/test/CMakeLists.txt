# Create an executable program qt4-widget-gfe-control-test
add_executable(qt4-widget-gfe-control-test qt4_widget_gfe_control_test.cc)
add_executable(qt4-widget-gfe-object-test qt4_widget_gfe_object_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(qt4-widget-gfe-control-test qt4_widget_gfe)
pods_use_pkg_config_packages(qt4-widget-gfe-object-test qt4_widget_gfe)

# make executable public
pods_install_executables(qt4-widget-gfe-control-test)
pods_install_executables(qt4-widget-gfe-object-test)
