#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)
add_definitions(-Wall -std=gnu99)

add_library(renderer_score SHARED renderer_score.cpp )
  
set_target_properties(renderer_score PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_score)
pods_install_headers(renderer_score.hpp DESTINATION renderer_drc)
  
pods_install_pkg_config_file(renderer_score
    LIBS -lrenderer_score
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_score
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames)
