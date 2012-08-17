#pkg_check_modules(BOT_CORE bot2-core)
#if(NOT BOT_CORE_FOUND)
#    message("bot2-core not found.  Not building paired-stereo-odometry")
#    return()
#endif()

add_executable(drc-paired-stereo-odometry
    jpeg-utils.c
    main.cpp
    paired-stereo-odometry.cpp
    visualization.cpp
    )

pods_use_pkg_config_packages(drc-paired-stereo-odometry
    glib-2.0
    lcm
    libfovis
    bot2-core
    bot2-param-client
    lcmtypes_fovis-bot2
    lcmtypes_drc_lcmtypes
    lcmtypes_bot2-core
    )

# link against the LCM types defined in ../lcmtypes/
target_link_libraries(drc-paired-stereo-odometry ${LCMTYPES_LIBS})

# use LCMGL if available
pkg_check_modules(LCMGL bot2-lcmgl-client)
if(LCMGL_FOUND)
    target_link_libraries(drc-paired-stereo-odometry ${LCMGL_LIBRARIES})
    include_directories(${LCMGL_INCLUDE_DIRS})
    add_definitions(-DUSE_LCMGL)
endif()

pods_install_executables(drc-paired-stereo-odometry)

target_link_libraries(drc-paired-stereo-odometry jpeg)
