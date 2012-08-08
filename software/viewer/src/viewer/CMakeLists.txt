
add_executable(drc-viewer main.cpp udp_util.c)

pods_use_pkg_config_packages(drc-viewer 
	bot2-vis  bot2-lcmgl-renderer bot2-frames-renderers  
      laser-util-renderer image-util-renderer path-util
      lcmtypes_drc_lcmtypes    renderer_drc 
      collections_renderer
      )
# lcm 	renderer_drc   )
# TODO: remove path util dependency

#scrollingplots_renderer

#kinect-renderer 

pods_install_executables(drc-viewer)
