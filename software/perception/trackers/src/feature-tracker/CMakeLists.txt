set(PACKAGE_NAME feature-tracker)

# Create a shared library
add_library(
  ${PACKAGE_NAME} SHARED
  KeyFrame.cpp
  CameraModel.cpp
  PoseEstimator.cpp
  )

# make the header public and install it
pods_install_headers(
  PyramidLevel.hpp
  KeyFrame.hpp
  CameraModel.hpp
  RansacGeneric.hpp
  PoseEstimator.hpp
  DESTINATION ${PACKAGE_NAME})

# make the library public
pods_install_libraries(${PACKAGE_NAME})

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES eigen3)
pods_use_pkg_config_packages(${PACKAGE_NAME} ${REQUIRED_PACKAGES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(${PACKAGE_NAME}
    CFLAGS
    LIBS -l${PACKAGE_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
