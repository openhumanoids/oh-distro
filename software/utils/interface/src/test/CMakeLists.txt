# Create an executable program interface-handler-test
add_executable(interface-handler-test interface_handler_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(interface-handler-test interface)

# make executable public
pods_install_executables(interface-handler-test)
