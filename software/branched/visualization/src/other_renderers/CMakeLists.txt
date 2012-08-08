find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_visualization SHARED 
    renderer_rwx.cpp
    renderer_wavefront_model.cpp
    renderer_rwx_model.cpp
    renderer_status.cpp
    renderer_localize.cpp
    renderer_groundtruth.cpp
    panorama_renderer.cpp
    scrollingplots_renderer.cpp
    )
set_target_properties(renderer_visualization PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_visualization)
pods_install_headers(renderer_rwx.hpp
    renderer_wavefront_model.hpp
    renderer_rwx_model.hpp
    renderer_status.hpp 
    renderer_localize.hpp
    renderer_groundtruth.hpp
    panorama_renderer.hpp
    scrollingplots_renderer.hpp 
    DESTINATION visualization)

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(renderer_visualization
    LIBS -lrenderer_visualization -ljpeg -l${LCMTYPES_LIBS}
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_visualization ${REQUIRED_LIBRARIES})


#####################################################################
#add_library(panorama_renderer SHARED panorama_renderer.cpp)
#pods_use_pkg_config_packages(panorama_renderer lcm bot2-vis)
#set_target_properties(panorama_renderer PROPERTIES SOVERSION 1)
#pods_install_libraries(panorama_renderer)
##pods_install_headers(panorama_renderer.hpp DESTINATION visualization)
#pods_install_pkg_config_file(panorama_renderer
#    LIBS -lpanorama_renderer
#    REQUIRES bot2-vis bot2-vis bot2-frames lcm
#    VERSION 0.0.1)
