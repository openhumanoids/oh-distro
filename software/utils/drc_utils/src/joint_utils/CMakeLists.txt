set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")



# make the header public
# install it to include/drc_utils
pods_install_headers(joint_utils.hpp DESTINATION drc_utils)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(drc-utils
    CFLAGS
    LIBS -ldrc-utils
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
