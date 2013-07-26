################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(kintinuous_pub)

add_library(kintinuous_pub SHARED kintinuous_pub.cpp)

pods_use_pkg_config_packages(kintinuous_pub lcm pcl_io-1.6 opencv lcmtypes_multisense)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(kintinuous_pub PROPERTIES SOVERSION 1)
pods_install_libraries(kintinuous_pub)
pods_install_headers(kintinuous_pub.hpp DESTINATION kintinuous_pub )

target_link_libraries(kintinuous_pub
    z)
pods_install_pkg_config_file(kintinuous_pub
    LIBS -lkintinuous_pub 
    REQUIRES pcl_io-1.6 pcl_common-1.6 lcmtypes_multisense
    VERSION 0.0.1)

