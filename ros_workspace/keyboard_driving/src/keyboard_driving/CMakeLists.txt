add_definitions(-Wall )##############################################




##############################################
add_executable (drc-keyboard-driving keyboard_driving.cpp)
target_link_libraries (drc-keyboard-driving curses    )
pods_use_pkg_config_packages(drc-keyboard-driving
   eigen3  glib-2.0  lcm bot2-core 
    path-util )
pods_install_executables(drc-keyboard-driving)
