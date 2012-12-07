# Create an executable program hello-test
add_executable(collision-detector-test collision_detector_test.cc)
add_executable(collision-object-test collision_object_test.cc)
add_executable(collision-object-box-test collision_object_box_test.cc)
add_executable(collision-object-cone-test collision_object_cone_test.cc)
add_executable(collision-object-cylinder-test collision_object_cylinder_test.cc)
add_executable(collision-object-sphere-test collision_object_sphere_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(collision-detector-test collision-detection)
pods_use_pkg_config_packages(collision-object-test collision-detection)
pods_use_pkg_config_packages(collision-object-box-test collision-detection)
pods_use_pkg_config_packages(collision-object-cone-test collision-detection)
pods_use_pkg_config_packages(collision-object-cylinder-test collision-detection)
pods_use_pkg_config_packages(collision-object-sphere-test collision-detection)

# make executable public
pods_install_executables(collision-detector-test)
pods_install_executables(collision-object-test)
pods_install_executables(collision-object-box-test)
pods_install_executables(collision-object-cone-test)
pods_install_executables(collision-object-cylinder-test)
pods_install_executables(collision-object-sphere-test)
