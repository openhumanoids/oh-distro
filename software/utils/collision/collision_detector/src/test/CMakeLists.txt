# Create an executable program hello-test
add_executable(collision-detector-test collision_detector_test.cc)
add_executable(collision-detector-point-cloud-filtering-test collision_detector_point_cloud_filtering_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(collision-detector-test collision_detector)
pods_use_pkg_config_packages(collision-detector-point-cloud-filtering-test collision_detector)

# make executable public
pods_install_executables(collision-detector-test)
pods_install_executables(collision-detector-point-cloud-filtering-test)
