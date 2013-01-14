# Create an executable program collision-object-gfe-test
add_executable(collision-object-gfe-test collision_object_gfe_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(collision-object-gfe-test collision_object_gfe)

# make executable public
pods_install_executables(collision-object-gfe-test)
