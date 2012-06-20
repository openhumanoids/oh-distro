
add_executable(drc-viewer main.cpp)

pods_use_pkg_config_packages(drc-viewer bot2-vis  bot2-lcmgl-renderer renderer_drc path-util
      lcmtypes_drc_lcmtypes)
# lcm 	renderer_drc   )
# TODO: remove path util dependency

#scrollingplots_renderer

#kinect-renderer 

pods_install_executables(drc-viewer)
