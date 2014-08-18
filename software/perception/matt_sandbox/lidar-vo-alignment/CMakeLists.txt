cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME dump-head-data)
include(cmake/pods.cmake)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wreturn-type -Wuninitialized -Wunused-variable -Wno-deprecated-declarations")

# create an executable, and make it public
set(REQUIRED_PACKAGES opencv drc-utils)
add_executable(${POD_NAME} src/${POD_NAME}.cpp)
pods_use_pkg_config_packages(${POD_NAME} ${REQUIRED_PACKAGES})
pods_install_executables(${POD_NAME})

set(TARGET_NAME feed-fovis)
set(REQUIRED_PACKAGES drc-utils opencv voconfig fovision)
add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
#target_link_libraries(${TARGET_NAME} boost_system boost_filesystem boost_regex)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})


set(TARGET_NAME integrate-scans)
set(REQUIRED_PACKAGES drc-utils)
add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})



set(TARGET_NAME run-vo-experiment)
set(REQUIRED_PACKAGES drc-utils opencv voconfig fovision)
add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})


set(TARGET_NAME align-point-clouds)
set(REQUIRED_PACKAGES
  pcl_common-1.7
  pcl_filters-1.7
  pcl_features-1.7
  pcl_io-1.7
  pcl_surface-1.7
)
add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})


set(TARGET_NAME data-blob-test)
set(REQUIRED_PACKAGES maps)
add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})


set(TARGET_NAME grab-lidar-cloud)
set(REQUIRED_PACKAGES drc-utils)
add_executable(${TARGET_NAME} src/${TARGET_NAME}.cpp)
pods_install_executables(${TARGET_NAME})
pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})
