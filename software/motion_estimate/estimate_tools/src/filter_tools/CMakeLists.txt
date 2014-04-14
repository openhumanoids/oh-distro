set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations -DNEBUG")


cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(filter_tools)


################


add_library(filter_tools  SHARED Filter.cpp HeavyLowPassFilter.cpp SignalTap.cpp)
pods_use_pkg_config_packages(filter_tools  ${PKG_CONFIG_DEPS}  
  eigen3)
set_target_properties(filter_tools PROPERTIES SOVERSION 1)
pods_install_libraries(filter_tools)
pods_install_headers( Filter.hpp HeavyLowPassFilter.hpp
              SignalTap.hpp
              DESTINATION estimate_tools)
pods_install_pkg_config_file(filter_tools
  LIBS -lfilter_tools
  REQUIRES 
  VERSION 0.0.1)


