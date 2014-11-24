find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
set(GLUT_LIBRARIES -lglut)

pkg_check_modules(BOT2_VIS bot2-vis)
if(NOT BOT2_VIS_FOUND)
    message("bot2-vis not found.  Not building libbot2 renderer")
    return()
endif(NOT BOT2_VIS_FOUND)

#add_definitions(-Wall -std=gnu99)

add_library(multisense-renderer SHARED 
    multisense_renderer.cpp ../multisense-glview/jpeg-utils-ijg.c
    )

target_link_libraries(multisense-renderer
    ${OPENGL_LIBRARIES}
    ${GTK2_LDFLAGS})

pods_use_pkg_config_packages(multisense-renderer
    bot2-vis opencv
    lcmtypes_multisense
    bot2-frames eigen3)

pods_install_headers(multisense_renderer.h 
    DESTINATION multisense)

pods_install_libraries(multisense-renderer)

pods_install_pkg_config_file(multisense-renderer
    LIBS -lmultisense-renderer
    REQUIRES bot2-vis bot2-frames 
    VERSION 0.0.1)

add_executable(multisense-viewer main.cpp)

target_link_libraries(multisense-viewer
    ${GLUT_LIBRARIES})

pods_use_pkg_config_packages(multisense-viewer
    glib-2.0
    bot2-vis
    bot2-frames
    bot2-param-client 
    lcm
    bot2-lcmgl-renderer
    multisense-renderer)

pods_install_libraries(multisense-renderer)
pods_install_executables(multisense-viewer)
