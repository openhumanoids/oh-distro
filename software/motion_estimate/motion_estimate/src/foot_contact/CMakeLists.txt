cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(foot_contact)

add_library(foot_contact  SHARED FootContact.cpp)
pods_use_pkg_config_packages(foot_contact 
  filter_tools 
  eigen3)
set_target_properties(foot_contact PROPERTIES SOVERSION 1)
pods_install_libraries(foot_contact)
pods_install_headers( FootContact.h 
         DESTINATION foot_contact)
pods_install_pkg_config_file(foot_contact
  LIBS -lfoot_contact -lfilter_tools
  VERSION 0.0.1)

