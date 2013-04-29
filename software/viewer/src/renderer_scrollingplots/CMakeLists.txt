#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)
add_definitions( -std=gnu99)

add_library(renderer_scrollingplots SHARED renderer_scrollingplots.c )
  
set_target_properties(renderer_scrollingplots PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_scrollingplots)
pods_install_headers(renderer_scrollingplots.h DESTINATION renderer_drc)
  
pods_install_pkg_config_file(renderer_scrollingplots
    LIBS -lrenderer_scrollingplots
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_scrollingplots
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames)




