# Create a shared library
set(LIB_NAME jpl-tags)
add_library(${LIB_NAME} SHARED
  fiducial_detector.c
  fiducial_stereo.c
)

# make the header public and install it
pods_install_headers(
  fiducial_detector.h
  fiducial_stereo.h
  fiducial_pose.h
  DESTINATION ${LIB_NAME})

# make the library public
pods_install_libraries(${LIB_NAME})

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES opencv)
pods_use_pkg_config_packages(${LIB_NAME} ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(${LIB_NAME}
    CFLAGS
    LIBS -l${LIB_NAME} -lgomp
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
