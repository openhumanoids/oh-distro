# Create an executable program opengl_scene_test
add_executable(opengl-scene-test opengl_scene_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-scene-test opengl_scene)

# make executable public
pods_install_executables(opengl-scene-test)
