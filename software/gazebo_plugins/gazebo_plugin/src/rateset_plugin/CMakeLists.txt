cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo REQUIRED)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(rateset_plugin SHARED  rateset_plugin.cc)
pods_install_libraries(rateset_plugin)
pods_use_pkg_config_packages(rateset_plugin eigen3)
#        pointcloud_lcm        pointcloud_vis        pointcloud_math)
target_link_libraries(rateset_plugin ${GAZEBO_LIBRARIES} lcm boost_thread)
pods_install_pkg_config_file(rateset_plugin
    CFLAGS
    LIBS -lrateset_plugin
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
