# Create a shared library libkinematics_model_gfe.so with a single source file
add_library(kinematics_model_gfe SHARED
    kinematics_model_gfe.cc)

# make the header public
# install it to include/kinematics_model_gfe
pods_install_headers(kinematics_model_gfe.h DESTINATION kinematics)

# make the library public
pods_install_libraries(kinematics_model_gfe)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES state_gfe state kinematics_model forward-kinematics orocos-kdl kdl-parser path-util urdf)
pods_use_pkg_config_packages(kinematics_model_gfe ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(kinematics_model_gfe
    CFLAGS
    LIBS -lkinematics_model_gfe
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
