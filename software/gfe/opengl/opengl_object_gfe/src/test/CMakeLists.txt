# Create an executable program hello-test
add_executable(opengl-object-gfe-test opengl_object_gfe_test.cc)
add_executable(opengl-object-trajectory-gfe-test opengl_object_trajectory_gfe_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-object-gfe-test opengl_object_gfe)
pods_use_pkg_config_packages(opengl-object-trajectory-gfe-test opengl_object_gfe)

# make executable public
pods_install_executables(opengl-object-gfe-test)
pods_install_executables(opengl-object-trajectory-gfe-test)
