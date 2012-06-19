# Create an executable program hello-test
add_executable(hello-test hello-test.c)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(hello-test hello)

# make executable public
pods_install_executables(hello-test)
