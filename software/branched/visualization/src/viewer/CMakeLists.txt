find_package(PkgConfig REQUIRED)
pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building mr-viewer2")
    return()
endif(NOT BOT2_VIS_FOUND)

pkg_check_modules(LCMTYPES_KINECT lcmtypes_kinect)
if(NOT LCMTYPES_KINECT_FOUND)
    message("lcmtypes_kinect not found.  Not building mr-viewer2")
    return()
endif(NOT LCMTYPES_KINECT_FOUND)

add_executable(mr-viewer2 main.cpp)
pods_use_pkg_config_packages(mr-viewer2 bot2-vis 
  bot2-lcmgl-renderer lcm  lcmtypes_kinect
  collections_renderer  renderer_visualization
   path-util)
# TODO: remove path util dependency

#scrollingplots_renderer

#kinect-renderer 

pods_install_executables(mr-viewer2)
