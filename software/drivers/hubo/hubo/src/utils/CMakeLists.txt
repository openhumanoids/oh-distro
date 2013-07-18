cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

add_executable(hubo-utils-listenSocket listenSocket.cpp)
pods_use_pkg_config_packages(hubo-utils-listenSocket lcm glib-2.0 PracticalSocket)

add_executable(hubo-utils-manualHubo manualHubo.cpp)
pods_use_pkg_config_packages(hubo-utils-manualHubo lcm glib-2.0 PracticalSocket)

add_executable(hubo-utils-manualLCM manualLCM.cpp)
pods_use_pkg_config_packages(hubo-utils-manualLCM lcm glib-2.0 PracticalSocket)

pods_install_executables(hubo-utils-listenSocket hubo-utils-manualHubo)
