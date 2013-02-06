cmake_minimum_required(VERSION 2.4.6)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(multisense_subscribe multisense_subscribe.cpp)
pods_use_pkg_config_packages(real_multisense_subscribe)
pods_install_executables( multisense_subscribe )

