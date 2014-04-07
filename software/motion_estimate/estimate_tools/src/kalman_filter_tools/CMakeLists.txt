set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(kalman_filter_tools)


################
add_library(kalman_filter_tools  SHARED kalman_filter.cpp simple_kalman_filter.cpp)
pods_use_pkg_config_packages(kalman_filter_tools  ${PKG_CONFIG_DEPS}  
  eigen3)
set_target_properties(kalman_filter_tools PROPERTIES SOVERSION 1)
pods_install_libraries(kalman_filter_tools)
pods_install_headers( kalman_filter.hpp simple_kalman_filter.hpp
              DESTINATION estimate_tools)
pods_install_pkg_config_file(kalman_filter_tools
  LIBS -lkalman_filter_tools
  REQUIRES 
  VERSION 0.0.1)


################
add_executable(se-kalman-filter-demo kalman_filter_demo.cpp )
pods_use_pkg_config_packages(se-kalman-filter-demo bot2-frames_cpp
    path-util filter_tools kalman_filter_tools 
    pointcloud_vis  )
target_link_libraries (se-kalman-filter-demo lcm pointcloud_math  )
pods_install_executables(se-kalman-filter-demo )
