add_executable(mav-rpy-translator rpy-translator.cpp)
pods_use_pkg_config_packages(mav-rpy-translator lcm lcmtypes_mav-lcmtypes bot2-core eigen3)
pods_install_executables(mav-rpy-translator)


