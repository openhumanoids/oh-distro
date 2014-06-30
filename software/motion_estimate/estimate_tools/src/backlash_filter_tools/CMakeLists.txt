set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(backlash_filter_tools)


################
add_library(backlash_filter_tools  SHARED backlash_filter.cpp)
pods_use_pkg_config_packages(backlash_filter_tools  ${PKG_CONFIG_DEPS}  
  eigen3)
set_target_properties(backlash_filter_tools PROPERTIES SOVERSION 1)
pods_install_libraries(backlash_filter_tools)
pods_install_headers( backlash_filter.hpp
              DESTINATION estimate_tools)
pods_install_pkg_config_file(backlash_filter_tools
  LIBS -lbacklash_filter_tools
  REQUIRES 
  VERSION 0.0.1)



