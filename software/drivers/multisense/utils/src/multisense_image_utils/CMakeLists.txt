set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x ")

################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(multisense_image_utils)

add_library(multisense_image_utils SHARED multisense_image_utils.cpp)

pods_use_pkg_config_packages(multisense_image_utils lcm   opencv)

# set the library API version.  Increment this every time the public API changes.
set_target_properties(multisense_image_utils PROPERTIES SOVERSION 1)
pods_install_libraries(multisense_image_utils)
pods_install_headers(multisense_image_utils.hpp DESTINATION multisense_image_utils)

target_link_libraries(multisense_image_utils
    z)
pods_install_pkg_config_file(multisense_image_utils
    LIBS -lmultisense_image_utils
    REQUIRES opencv
    VERSION 0.0.1)

