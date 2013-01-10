add_definitions(-Wall )##############################################




##############################################
add_executable (drc-ncurses-example ncurses_example.cpp)
target_link_libraries (drc-ncurses-example curses    )
pods_use_pkg_config_packages(drc-ncurses-example
   eigen3  glib-2.0  lcm bot2-core 
    path-util )
pods_install_executables(drc-ncurses-example)
