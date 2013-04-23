cmake_minimum_required(VERSION 2.8)

set(PACKAGE_NAME feature-tracking)

# Create a shared library
add_library(
  ${PACKAGE_NAME} SHARED
  PatchUtils.cpp
  PointMatcher.cpp
  KeyFrame.cpp
  CameraModel.cpp
  StereoCamera.cpp
  PoseEstimator.cpp
  FeatureBasedTracker.cpp
  BotUtils.cpp
  )

# make the header public and install it
pods_install_headers(
  PatchUtils.hpp
  PointMatcher.hpp
  KeyFrame.hpp
  CameraModel.hpp
  StereoCamera.hpp
  RansacGeneric.hpp
  PoseEstimator.hpp
  FeatureBasedTracker.hpp
  BotUtils.hpp
  DESTINATION ${PACKAGE_NAME})

# make the library public
pods_install_libraries(${PACKAGE_NAME})

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES eigen3 opencv bot2-param-client)
pods_use_pkg_config_packages(${PACKAGE_NAME} ${REQUIRED_PACKAGES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(${PACKAGE_NAME}
    CFLAGS
    LIBS -l${PACKAGE_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# unit tests
set(EXE_NAME feature-tracking-test)
add_executable(${EXE_NAME} TestMain.cpp)
pods_use_pkg_config_packages(${EXE_NAME} feature-tracking)
pods_install_executables(${EXE_NAME})

# standalone tracker (to be integrated with object-tracker infrastructure)
set(EXE_NAME feature-tracker)
add_executable(${EXE_NAME} TrackerMain.cpp)
pods_use_pkg_config_packages(${EXE_NAME} feature-tracking zlib bot2-frames
  bot2-param-client)
pods_install_executables(${EXE_NAME})
