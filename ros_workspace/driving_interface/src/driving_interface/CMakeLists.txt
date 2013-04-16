add_definitions(-Wall )##############################################


##############################################
add_executable (drc-driving-interface main.cpp)
target_link_libraries (drc-driving-interface curses)
pods_use_pkg_config_packages(drc-driving-interface
   eigen3  glib-2.0  lcm bot2-core 
    path-util )
pods_install_executables(drc-driving-interface)
