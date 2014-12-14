# Create a shared library libhello.so with a single source file
add_library(kinematics_model SHARED
            kinematics_model.cc
            kinematics_model_urdf.cc)

# make the header public
# install it to include/hello
pods_install_headers(kinematics_model.h
                      kinematics_model_urdf.h
                      DESTINATION kinematics)

# make the library public
pods_install_libraries(kinematics_model)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES forward-kinematics orocos-kdl kdl-parser path-util urdf)
pods_use_pkg_config_packages(kinematics_model ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(kinematics_model
    CFLAGS
    LIBS -lkinematics_model
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
