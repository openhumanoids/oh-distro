#build the viewer

add_executable(mav-mapping-viewer 
    mapping_viewer.cpp
    )
pods_use_pkg_config_packages(mav-mapping-viewer 
    bot2-vis 
    bot2-lcmgl-renderer 
    bot2-frames-renderers
    laser-util-renderer
    octomap-renderer
    occ-map-renderers
    path-util
    mav-mapping-renderers)
pods_install_executables(mav-mapping-viewer)


