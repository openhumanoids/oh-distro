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

# Create a shared library libhello.so with a single source file
add_library(hokuyospin_plugin SHARED
    hokuyospin_plugin.cc)

# make the library public
pods_install_libraries(hokuyospin_plugin)

# uncomment these lines to link against another library via pkg-config
pods_use_pkg_config_packages(hokuyospin_plugin eigen3)
target_link_libraries(hokuyospin_plugin ${GAZEBO_LIBRARIES} lcm boost_thread)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(hokuyospin_plugin
    CFLAGS
    LIBS -lhokuyospin_plugin
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
