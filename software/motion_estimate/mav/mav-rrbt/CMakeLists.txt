cmake_minimum_required(VERSION 2.6.0)

file(GLOB hpp_files src/*.hpp)

set(POD_NAME rrbt)

include(cmake/pods.cmake)

pods_install_headers(${hpp_files} DESTINATION rrbt)
pods_install_pkg_config_file(rrbt
    LIBS m
    REQUIRES glib-2.0 lcm bot2-core bot2-lcmgl-client eigen-utils kdtree eigen3
    VERSION 0.0.1)

add_subdirectory(src/rrbt-test)


