add_definitions(-Wall )##############################################




##############################################
add_executable (drc-frames-changer frames_changer.cpp)
target_link_libraries (drc-frames-changer curses    )
pods_use_pkg_config_packages(drc-frames-changer
   eigen3  glib-2.0  lcm bot2-core 
    path-util )
pods_install_executables(drc-frames-changer)
