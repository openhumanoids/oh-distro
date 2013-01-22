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


#####################################################################
add_library(renderer_maps SHARED 
  renderer_maps.cpp)
  
set_target_properties(renderer_maps PROPERTIES SOVERSION 1)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(renderer_maps
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames
  pcl_common-1.6 octomap zlib maps)

pods_install_headers(renderer_maps.hpp
  DESTINATION renderer_maps)

pods_install_libraries(renderer_maps)
  
pods_install_pkg_config_file(renderer_maps
    LIBS -lrenderer_maps
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)



#####################################################################
# add_library(renderer_depthmap SHARED 
#   renderer_depthmap.cpp)
  
# set_target_properties(renderer_depthmap PROPERTIES SOVERSION 1)

# pods_use_pkg_config_packages(renderer_depthmap
#   drc-utils lcmtypes_drc_lcmtypes eigen3
#   bot2-core bot2-vis bot2-frames)

# pods_install_headers(renderer_depthmap.hpp
#   DESTINATION renderer_depthmap)

# pods_install_libraries(renderer_depthmap)
  
# pods_install_pkg_config_file(renderer_depthmap
#     LIBS -lrenderer_depthmap 
#     REQUIRES bot2-core bot2-vis bot2-frames 
#     VERSION 0.0.1)

