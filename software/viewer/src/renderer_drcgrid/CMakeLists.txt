#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)
add_definitions(-Wall -std=gnu99)

add_library(renderer_drcgrid SHARED renderer_drcgrid.c )
  
set_target_properties(renderer_drcgrid PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_drcgrid)
pods_install_headers(renderer_drcgrid.h DESTINATION renderer_drc)
  
pods_install_pkg_config_file(renderer_drcgrid
    LIBS -lrenderer_drcgrid
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_drcgrid
  bot2-core bot2-vis bot2-frames)
#  lcmtypes_drc_lcmtypes eigen3




