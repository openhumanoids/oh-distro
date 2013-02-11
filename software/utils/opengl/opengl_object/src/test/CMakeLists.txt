# Create an executable program opengl-object-test
add_executable(opengl-object-test opengl_object_test.cc)
add_executable(opengl-object-box-test opengl_object_box_test.cc)
add_executable(opengl-object-sphere-test opengl_object_sphere_test.cc)
add_executable(opengl-object-cylinder-test opengl_object_cylinder_test.cc)
add_executable(opengl-object-torus-test opengl_object_torus_test.cc)
add_executable(opengl-object-dae-test opengl_object_dae_test.cc)
add_executable(opengl-object-coordinate-axis-test opengl_object_coordinate_axis_test.cc)
add_executable(opengl-object-collision-detector-test opengl_object_collision_detector_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-object-test opengl_object)
pods_use_pkg_config_packages(opengl-object-box-test opengl_object)
pods_use_pkg_config_packages(opengl-object-sphere-test opengl_object)
pods_use_pkg_config_packages(opengl-object-cylinder-test opengl_object)
pods_use_pkg_config_packages(opengl-object-torus-test opengl_object)
pods_use_pkg_config_packages(opengl-object-dae-test opengl_object)
pods_use_pkg_config_packages(opengl-object-coordinate-axis-test opengl_object)
pods_use_pkg_config_packages(opengl-object-collision-detector-test opengl_object)

# make executable public
pods_install_executables(opengl-object-test)
pods_install_executables(opengl-object-box-test)
pods_install_executables(opengl-object-sphere-test)
pods_install_executables(opengl-object-cylinder-test)
pods_install_executables(opengl-object-torus-test)
pods_install_executables(opengl-object-dae-test)
pods_install_executables(opengl-object-coordinate-axis-test)
pods_install_executables(opengl-object-collision-detector-test)
