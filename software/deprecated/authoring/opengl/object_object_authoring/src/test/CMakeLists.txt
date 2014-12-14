# Create an executable program opengl-object-affordance-test
add_executable(opengl-object-constraint-sequence-test opengl_object_constraint_sequence_test.cc)
add_executable(opengl-object-constraint-task-space-region-test opengl_object_constraint_task_space_region_test.cc)
add_executable(opengl-object-affordance-test opengl_object_affordance_test.cc)
add_executable(opengl-object-affordance-collection-test opengl_object_affordance_collection_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(opengl-object-constraint-sequence-test opengl_object_authoring)
pods_use_pkg_config_packages(opengl-object-constraint-task-space-region-test opengl_object_authoring)
pods_use_pkg_config_packages(opengl-object-affordance-test opengl_object_authoring)
pods_use_pkg_config_packages(opengl-object-affordance-collection-test opengl_object_authoring)

# make executable public
pods_install_executables(opengl-object-constraint-sequence-test)
pods_install_executables(opengl-object-constraint-task-space-region-test)
pods_install_executables(opengl-object-affordance-test)
pods_install_executables(opengl-object-affordance-collection-test)
