add_executable(drc-controller-contact-passthrough contact-passthrough.cpp)
pods_use_pkg_config_packages(drc-controller-contact-passthrough
   lcmtypes_drc_lcmtypes lcmtypes_bot2-core lcm)
pods_install_executables(drc-controller-contact-passthrough )
