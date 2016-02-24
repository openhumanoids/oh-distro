###############
add_executable(simple_isam_demo simple_isam_demo.cpp)
pods_use_pkg_config_packages(simple_isam_demo
    pronto_vis pronto_math  eigen3 )
target_link_libraries (simple_isam_demo isam cholmod  )
pods_install_executables(simple_isam_demo)
