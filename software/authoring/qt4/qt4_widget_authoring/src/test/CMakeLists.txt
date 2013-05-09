# Create an executable program qt4-widget-authoring-test
add_executable(qt4-widget-double-slider-test qt4_widget_double_slider_test.cc)
add_executable(qt4-widget-constraint-editor-test qt4_widget_constraint_editor_test.cc)
add_executable(qt4-widget-constraint-task-space-region-editor-test qt4_widget_constraint_task_space_region_editor_test.cc)
add_executable(qt4-widget-authoring-test qt4_widget_authoring_test.cc)
add_executable(qt4-widget-authoring-lcm-interface-test qt4_widget_authoring_lcm_interface_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(qt4-widget-double-slider-test qt4_widget_authoring)
pods_use_pkg_config_packages(qt4-widget-constraint-editor-test qt4_widget_authoring)
pods_use_pkg_config_packages(qt4-widget-constraint-task-space-region-editor-test qt4_widget_authoring)
pods_use_pkg_config_packages(qt4-widget-authoring-test qt4_widget_authoring)
pods_use_pkg_config_packages(qt4-widget-authoring-lcm-interface-test qt4_widget_authoring)

# make executable public
pods_install_executables(qt4-widget-double-slider-test)
pods_install_executables(qt4-widget-constraint-editor-test)
pods_install_executables(qt4-widget-constraint-task-space-region-editor-test)
pods_install_executables(qt4-widget-authoring-test)
pods_install_executables(qt4-widget-authoring-lcm-interface-test)
