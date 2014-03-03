cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(foot_contact)

add_library(foot_contact  SHARED FootContact.cpp)
pods_use_pkg_config_packages(foot_contact 
  eigen3)
set_target_properties(foot_contact PROPERTIES SOVERSION 1)
pods_install_libraries(foot_contact)
pods_install_headers( FootContact.h Footsteps.h 
         TwoLegsEstimate_types.h
         DESTINATION foot_contact)
pods_install_pkg_config_file(foot_contact
  LIBS -lfoot_contact
  VERSION 0.0.1)

