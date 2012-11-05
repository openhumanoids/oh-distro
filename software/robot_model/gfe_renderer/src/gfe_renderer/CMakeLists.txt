add_definitions(-std=gnu99)

add_executable(gfe-viewer
    main.c
    gfe_renderer.c)

pods_use_pkg_config_packages(gfe-viewer bot2-vis)

pods_install_executables(gfe-viewer)
