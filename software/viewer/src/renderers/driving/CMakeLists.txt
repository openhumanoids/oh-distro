#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)
add_definitions(-Wall -std=gnu99)

add_library(renderer_driving SHARED renderer_driving.cpp )
  
set_target_properties(renderer_driving PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_driving)
pods_install_headers(renderer_driving.hpp DESTINATION renderer_driving)
  
pods_install_pkg_config_file(renderer_driving
    LIBS -lrenderer_driving
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_driving
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames)
