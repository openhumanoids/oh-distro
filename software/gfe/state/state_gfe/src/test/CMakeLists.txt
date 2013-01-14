# Create an executable program state-gfe-test
add_executable(state-gfe-test state_gfe_test.cc)
add_executable(state-gfe-joint-test state_gfe_joint_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(state-gfe-test state_gfe)
pods_use_pkg_config_packages(state-gfe-joint-test state_gfe)

# make executable public
pods_install_executables(state-gfe-test)
pods_install_executables(state-gfe-joint-test)
