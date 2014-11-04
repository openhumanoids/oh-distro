set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(kalman_filter_tools)

################
# Joint Level Filters
add_executable(test-multisense-image-filtering test-multisense-image-filtering.cpp )
pods_use_pkg_config_packages(test-multisense-image-filtering
     multisense_image_utils   )
# bot2-frames_cpp   pronto_vis  )
target_link_libraries (test-multisense-image-filtering lcm z)
#pronto_math  )
pods_install_executables(test-multisense-image-filtering )


