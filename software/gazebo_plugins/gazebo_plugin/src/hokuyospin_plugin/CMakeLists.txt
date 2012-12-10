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

add_library(hokuyospin_plugin SHARED  hokuyospin_plugin.cc)
pods_install_libraries(hokuyospin_plugin)
pods_use_pkg_config_packages(hokuyospin_plugin eigen3)
target_link_libraries(hokuyospin_plugin ${GAZEBO_LIBRARIES} lcm boost_thread)
pods_install_pkg_config_file(hokuyospin_plugin
    CFLAGS
    LIBS -lhokuyospin_plugin
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

add_library(hokuyoright_plugin SHARED  hokuyoright_plugin.cc)
pods_install_libraries(hokuyoright_plugin)
pods_use_pkg_config_packages(hokuyoright_plugin eigen3)
target_link_libraries(hokuyoright_plugin ${GAZEBO_LIBRARIES} lcm boost_thread)
pods_install_pkg_config_file(hokuyoright_plugin
    CFLAGS
    LIBS -lhokuyoright_plugin
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
