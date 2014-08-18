cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# find gazebo
include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo QUIET)
else()
  message(FATAL_ERROR "pkg-config is required; please install it")
endif()
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

if(GAZEBO_FOUND)
  add_library(atlas_push_plugin SHARED  atlas_push_plugin.cc)
  pods_install_libraries(atlas_push_plugin)
  target_link_libraries(atlas_push_plugin ${GAZEBO_LIBRARIES} lcm boost_thread)
  pods_install_pkg_config_file(atlas_push_plugin
    CFLAGS
    LIBS -latlas_push_plugin
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

else()
  message("Gazebo was not found - skipping atlas_push_plugin")
endif()
