set(PACKAGE_NAME renderer_cam_thumb_drc)

add_definitions(-std=gnu99)
add_library(${PACKAGE_NAME} SHARED
    renderer_cam_thumb_drc.c)

target_link_libraries(${PACKAGE_NAME} ${OPENGL_LIBRARIES})

set(REQUIRED_LIBS image-utils bot2-vis bot2-param-client bot2-frames lcmtypes_multisense)

pods_use_pkg_config_packages(${PACKAGE_NAME} ${REQUIRED_LIBS})

pods_install_headers(renderer_cam_thumb_drc.h DESTINATION ${PACKAGE_NAME})
pods_install_libraries(${PACKAGE_NAME})
pods_install_pkg_config_file(${PACKAGE_NAME}
    CFLAGS
    LIBS -l${PACKAGE_NAME}
    REQUIRES ${REQUIRED_LIBS}
    VERSION 0.0.1)
