# Create an executable program state-test
add_executable(state-test state_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(state-test state)

# make executable public
pods_install_executables(state-test)
