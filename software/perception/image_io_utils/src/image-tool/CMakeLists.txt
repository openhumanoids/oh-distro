add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(image-tool)

add_executable(drc-image-tool image-tool.cpp  )
pods_use_pkg_config_packages(drc-image-tool  jpeg-utils
    lcmtypes_bot2-core opencv image_io_utils)

target_link_libraries(drc-image-tool     z     )

pods_install_executables(drc-image-tool )

