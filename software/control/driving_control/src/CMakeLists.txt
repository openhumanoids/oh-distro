#add_definitions(-std=gnu99)

add_executable(drc-driving-control main.cpp)

target_link_libraries (drc-rrtstar 
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(drc-driving-control
    lcm 
    glib-2.0 
    bot2-core
    bot2-param-client
    bot2-lcmgl-client
    bot2-frames
    occ-map
    lcmtypes_drc_lcmtypes
    lcmtypes_occ-map)


# make exeutables public
pods_install_executables(drc-driving-control)
