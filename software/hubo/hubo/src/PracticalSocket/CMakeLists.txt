cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(PracticalSocket)

add_library(PracticalSocket SHARED PracticalSocket.cpp)
#pods_use_pkg_config_packages(PracticalSocket lcm)
# set the library API version.  Increment this every time the public API changes.
set_target_properties(PracticalSocket PROPERTIES SOVERSION 1)
pods_install_libraries(PracticalSocket)
pods_install_headers(PracticalSocket.h DESTINATION hubo)

pods_install_pkg_config_file(PracticalSocket
    LIBS -lPracticalSocket
    VERSION 0.0.1)
#    REQUIRES lcm 
