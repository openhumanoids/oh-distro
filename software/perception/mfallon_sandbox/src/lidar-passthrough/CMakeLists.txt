add_executable(drc-lidar-passthrough lidar-passthrough.cpp)
pods_use_pkg_config_packages(drc-lidar-passthrough collision_detection lcm bot2-lcmgl-client )
pods_install_executables(drc-lidar-passthrough)
