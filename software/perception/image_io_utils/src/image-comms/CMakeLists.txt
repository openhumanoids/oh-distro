cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(image-comms)

add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

# image transmit
add_executable(drc-image-transmit ImageTransmit.cpp)
pods_use_pkg_config_packages(drc-image-transmit drc-utils opencv)
target_link_libraries(drc-image-transmit)
pods_install_executables(drc-image-transmit)

# image receive
add_executable(drc-image-receive ImageReceive.cpp)
pods_use_pkg_config_packages(drc-image-receive drc-utils opencv)
target_link_libraries(drc-image-receive)
pods_install_executables(drc-image-receive)
