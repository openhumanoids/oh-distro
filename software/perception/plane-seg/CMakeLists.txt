cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME plane-seg)
include(cmake/pods.cmake)

add_definitions(-std=c++1y -Wall)

# create library
set(LIB_NAME ${POD_NAME})
set(REQUIRED_PACKAGES
  eigen3
  pcl_io-1.7
  pcl_surface-1.7
  pcl_segmentation-1.7
)
add_library(${LIB_NAME} SHARED
  src/RobustNormalEstimator.cpp
  src/IncrementalPlaneEstimator.cpp
  src/PlaneSegmenter.cpp
  src/RectangleFitter.cpp
  src/BlockFitter.cpp
)
pods_install_headers(
  src/Types.hpp
  src/RobustNormalEstimator.hpp
  src/IncrementalPlaneEstimator.hpp
  src/PlaneSegmenter.hpp
  src/RectangleFitter.hpp
  src/BlockFitter.hpp
  DESTINATION ${LIB_NAME}
)
pods_install_libraries(${LIB_NAME})
pods_use_pkg_config_packages(${LIB_NAME} ${REQUIRED_PACKAGES})
pods_install_pkg_config_file(${LIB_NAME}
    CFLAGS -std=c++0x -Wall
    LIBS -l${LIB_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# create an executable, and make it public
set(APP_NAME ${POD_NAME}-test)
add_executable(${APP_NAME} src/${APP_NAME}.cpp)
target_link_libraries(${APP_NAME} boost_system)
pods_use_pkg_config_packages(${APP_NAME} ${LIB_NAME})
pods_install_executables(${APP_NAME})
