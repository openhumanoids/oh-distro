# Create an executable program constraint-test
add_executable(constraint-test constraint_test.cc)
add_executable(constraint-sequence-test constraint_sequence_test.cc)
add_executable(constraint-task-space-region-test constraint_task_space_region_test.cc)
add_executable(constraint-configuration-test constraint_configuration_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(constraint-test constraint_authoring)
pods_use_pkg_config_packages(constraint-sequence-test constraint_authoring)
pods_use_pkg_config_packages(constraint-task-space-region-test constraint_authoring)
pods_use_pkg_config_packages(constraint-configuration-test constraint_authoring)

# make executable public
pods_install_executables(constraint-test)
pods_install_executables(constraint-sequence-test)
pods_install_executables(constraint-task-space-region-test)
pods_install_executables(constraint-configuration-test)
