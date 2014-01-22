# Create a shared library libisamslam.so with a single source file
add_library(isamslam SHARED
    IsamSlam.cpp SlamPose.cpp)

# make the header public
# install it to include/isamslam
pods_install_headers(IsamSlam.hpp SlamPose.hpp DESTINATION isamslam)

# make the library public
pods_install_libraries(isamslam)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES scanmatch  occ-map lcmtypes_isam_slam bot2-lcmgl-client)
pods_use_pkg_config_packages(isamslam ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(isamslam
    CFLAGS
    LIBS -lisamslam -lisam -lcholmod
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)




add_executable(isam-slam isam_slam.cpp)

pods_use_pkg_config_packages(isam-slam  bot2-core bot2-lcmgl-client isamslam)
pods_install_executables(isam-slam)
