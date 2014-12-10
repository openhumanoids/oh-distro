# Create an executable program hello-test
add_executable(opengl-object-gfe-test opengl_object_gfe_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-object-gfe-test opengl_object_gfe)

# make executable public
pods_install_executables(opengl-object-gfe-test)
