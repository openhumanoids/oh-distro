cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(foot_contact_alt)

add_library(foot_contact_alt  SHARED FootContactAlt.cpp)
pods_use_pkg_config_packages(foot_contact_alt 
  filter_tools 
  eigen3)
set_target_properties(foot_contact_alt PROPERTIES SOVERSION 1)
pods_install_libraries(foot_contact_alt)
pods_install_headers( FootContactAlt.h 
         DESTINATION foot_contact_alt)
pods_install_pkg_config_file(foot_contact_alt
  LIBS -lfoot_contact_alt -lfilter_tools
  VERSION 0.0.1)

