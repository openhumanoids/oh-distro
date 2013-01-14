# Create an executable program hello-test
add_executable(kinematics-model-test kinematics_model_test.cc)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(kinematics-model-test kinematics_model)

# make executable public
pods_install_executables(kinematics-model-test)
