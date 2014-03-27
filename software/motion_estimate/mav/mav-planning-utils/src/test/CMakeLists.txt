add_executable(dubins-test dubins-test.cpp)
pods_use_pkg_config_packages(dubins-test mav-planning-utils)
pods_install_executables(dubins-test)

add_executable(poly-test poly-test.cpp)
pods_use_pkg_config_packages(poly-test mav-planning-utils)
pods_install_executables(poly-test)

add_executable(dubins-poly-test dubins-poly-test.cpp)
pods_use_pkg_config_packages(dubins-poly-test mav-planning-utils)
pods_install_executables(dubins-poly-test)
