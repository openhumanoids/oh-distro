find_package(PkgConfig REQUIRED)
add_definitions(-Wall)

#####################################################################
add_library(renderer_heightmap SHARED 
  renderer_heightmap.cpp)
  
set_target_properties(renderer_heightmap PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_heightmap
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames)

pods_install_headers(renderer_heightmap.hpp
  DESTINATION renderer_heightmap)

pods_install_libraries(renderer_heightmap)
  
pods_install_pkg_config_file(renderer_heightmap
    LIBS -lrenderer_heightmap 
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

