add_definitions(-g)


add_executable(grapher grapher.c)
target_link_libraries(grapher lcm )
pods_use_pkg_config_packages(grapher lcm lcmtypes_vicon-drc)
pods_install_executables(grapher)


