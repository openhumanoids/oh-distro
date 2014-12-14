cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME kdl_parser)
include(cmake/pods.cmake)

# Create a shared library libkdl_parser.so with a single source file
add_library(kdl-parser SHARED src/kdl_parser.cpp)

# make the header public
# install it to include/urdf_interface
pods_install_headers(include/kdl_parser/kdl_parser.hpp DESTINATION kdl_parser)

# make the library public
pods_install_libraries(kdl-parser)

# uncomment these lines to link against another library via pkg-config
set(REQUIRED_PACKAGES orocos-kdl urdf tinyxml)
pods_use_pkg_config_packages(kdl-parser ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(kdl-parser
    CFLAGS
    LIBS -lkdl-parser
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# Create executable programs
add_executable(check_kdl_parser src/check_kdl_parser.cpp)
pods_use_pkg_config_packages(check_kdl_parser kdl-parser ${REQUIRED_PACKAGES})
pods_install_executables(check_kdl_parser)


