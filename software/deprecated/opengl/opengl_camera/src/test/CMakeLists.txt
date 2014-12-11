# Create an executable program opengl-camera-test
add_executable(opengl-camera-test opengl_camera_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-camera-test opengl_camera)

# make executable public
pods_install_executables(opengl-camera-test)
