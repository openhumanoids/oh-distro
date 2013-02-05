add_executable(drc-image-splitter image-splitter.cpp  )
pods_use_pkg_config_packages(drc-image-splitter
                lcmtypes_bot2-core
                opencv)
pods_install_executables(drc-image-splitter )
