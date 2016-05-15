set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(sicklms5xx)


################
add_executable(sick-lcm sicklms5xx.cpp)
pods_use_pkg_config_packages(sick-lcm 
    path-util ${REQUIRED_PACKAGES}
    )
target_link_libraries (sick-lcm lcm
                        sicklms5xx)
pods_install_executables(sick-lcm )
