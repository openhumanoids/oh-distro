#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)

add_library(renderer_walking SHARED renderer_walking.cpp )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
  
set_target_properties(renderer_walking PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_walking)
pods_install_headers(renderer_walking.hpp DESTINATION renderer_drc)
  
pods_install_pkg_config_file(renderer_walking
    LIBS -lrenderer_walking
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_walking
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames maps)
