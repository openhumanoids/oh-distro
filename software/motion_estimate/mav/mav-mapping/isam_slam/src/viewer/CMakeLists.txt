#build the viewer
add_executable(isam-slam-viewer isam_slam_viewer.cpp)

pods_use_pkg_config_packages(isam-slam-viewer bot2-vis bot2-lcmgl-renderer occ-map-renderers)

pods_install_executables(isam-slam-viewer)