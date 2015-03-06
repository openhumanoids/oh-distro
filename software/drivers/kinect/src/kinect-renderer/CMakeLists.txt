find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
set(GLUT_LIBRARIES -lglut)

pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building libbot2 renderer")
    return()
endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

add_library(kinect-renderer SHARED 
    kinect_renderer.c
    jpeg-utils-ijg.c
    )

target_link_libraries(kinect-renderer
    ${OPENGL_LIBRARIES}
    ${GTK2_LDFLAGS}
    ${LCMTYPES_LIBS})

pods_use_pkg_config_packages(kinect-renderer
    bot2-vis
    bot2-frames
    kinect-utils)

pods_install_headers(kinect_renderer.h 
    DESTINATION kinect)

pods_install_libraries(kinect-renderer)

pods_install_pkg_config_file(kinect-renderer
    LIBS -lkinect-renderer -ljpeg -l${LCMTYPES_LIBS}
    REQUIRES bot2-vis bot2-frames kinect-utils
    VERSION 0.0.1)


add_executable(kinect-viewer main.c)

target_link_libraries(kinect-viewer
    ${GLUT_LIBRARIES})

pods_use_pkg_config_packages(kinect-viewer
    glib-2.0
    bot2-vis
    bot2-frames
    bot2-param-client 
    lcm
    kinect-utils
    kinect-renderer)

pods_install_libraries(kinect-renderer)

pods_install_executables(kinect-viewer)
