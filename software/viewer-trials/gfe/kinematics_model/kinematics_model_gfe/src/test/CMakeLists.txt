# Create an executable program kinematics-model-gfe-test
add_executable(kinematics-model-gfe-test kinematics_model_gfe_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(kinematics-model-gfe-test kinematics_model_gfe)

# make executable public
pods_install_executables(kinematics-model-gfe-test)
