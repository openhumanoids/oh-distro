add_library(jpeg-utils SHARED jpeg-utils.c jpeg-utils-ijg.c)

target_link_libraries(jpeg-utils jpeg ${LCMTYPES_LIBS})

set_target_properties(jpeg-utils PROPERTIES SOVERSION 1)
pods_install_libraries(jpeg-utils)
pods_install_headers(jpeg-utils-ijg.h jpeg-utils.h DESTINATION jpeg-utils)
pods_install_pkg_config_file(jpeg-utils
    LIBS -ljpeg-utils
    VERSION 0.0.1)

