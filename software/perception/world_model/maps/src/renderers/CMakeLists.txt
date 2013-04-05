find_package(PkgConfig REQUIRED)
add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

#####################################################################
add_library(renderer_data_control SHARED 
  DataControlRenderer.cpp
  RendererBase.cpp)
  
set_target_properties(renderer_data_control PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_data_control
  drc-utils lcmtypes_drc_lcmtypes bot2-vis gtkmm-2.4)

pods_install_headers(DataControlRenderer.hpp
  DESTINATION renderer_data_control)

pods_install_libraries(renderer_data_control)
  
pods_install_pkg_config_file(renderer_data_control
    LIBS -lrenderer_data_control
    REQUIRES lcmtypes_drc_lcmtypes bot2-vis drc-utils
    VERSION 0.0.1)

#####################################################################
add_library(renderer_maps SHARED 
  MapsRenderer.cpp
  MeshRenderer.cpp)
  
set_target_properties(renderer_maps PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_maps
  bot2-vis maps gtkmm-2.4 jpeg-utils)

pods_install_headers(
  MapsRenderer.hpp
  DESTINATION renderer_maps)

pods_install_libraries(renderer_maps)
  
pods_install_pkg_config_file(renderer_maps
    LIBS -lrenderer_maps
    REQUIRES bot2-vis maps
    VERSION 0.0.1)


#####################################################################
add_library(renderer_lidartest SHARED 
  RendererBase.cpp
  MeshRenderer.cpp
  LidarTestRenderer.cpp
)

set_target_properties(renderer_lidartest PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_lidartest
  lcmtypes_drc_lcmtypes eigen3
  bot2-core bot2-vis bot2-frames jpeg-utils gtkmm-2.4 maps)

pods_install_headers(LidarTestRenderer.hpp RendererBase.hpp
  DESTINATION renderer_lidartest)

pods_install_libraries(renderer_lidartest)
  
pods_install_pkg_config_file(renderer_lidartest
    LIBS -lrenderer_lidartest
    REQUIRES bot2-core bot2-vis bot2-frames
    VERSION 0.0.1)

