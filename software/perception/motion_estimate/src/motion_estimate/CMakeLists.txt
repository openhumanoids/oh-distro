#pkg_check_modules(BOT_CORE bot2-core)
#if(NOT BOT_CORE_FOUND)
#    message("bot2-core not found.  Not building motion-estimate")
#    return()
#endif()

add_executable(drc-motion-estimate
    jpeg-utils.c
    main.cpp
    modified-stereo-odometry.cpp
    visualization.cpp
    )

pods_use_pkg_config_packages(drc-motion-estimate
    glib-2.0
    lcm
    libfovis
    bot2-core
    bot2-param-client
    bot2-frames # mfallon
    lcmtypes_fovis-bot2
    lcmtypes_bot2-core
    lcmtypes_microstrain_comm
    )

# link against the LCM types defined in ../lcmtypes/
target_link_libraries(drc-motion-estimate ${LCMTYPES_LIBS})

# use LCMGL if available
pkg_check_modules(LCMGL bot2-lcmgl-client)
if(LCMGL_FOUND)
    target_link_libraries(drc-motion-estimate ${LCMGL_LIBRARIES})
    include_directories(${LCMGL_INCLUDE_DIRS})
    add_definitions(-DUSE_LCMGL)
endif()

pods_install_executables(drc-motion-estimate)

target_link_libraries(drc-motion-estimate jpeg fovis)
