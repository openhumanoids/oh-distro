find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_drc SHARED 
  humanoid_renderer/renderer_humanoid.cpp
  humanoid_renderer/RobotStateListener.cpp
  renderer_navigation.cpp
  )
set_target_properties(renderer_drc PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_drc)
pods_install_headers(humanoid_renderer/renderer_humanoid.hpp
  humanoid_renderer/RobotStateListener.hpp
  renderer_navigation.hpp
  DESTINATION renderer_drc)

# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_HUMANOID)
    message("REQUIRED_PACKAGES_HUMANOID already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_HUMANOID)

set(REQUIRED_PACKAGES_HUMANOID urdf orocos-kdl kdl-parser forward-kinematics)


pods_install_pkg_config_file(renderer_drc
    LIBS -lrenderer_drc -ljpeg 
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_drc
  lcmtypes_drc_lcmtypes
  bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES_HUMANOID})





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
