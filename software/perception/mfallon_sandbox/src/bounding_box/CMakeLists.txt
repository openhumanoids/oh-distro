add_executable(test-bounding-box bounding_box.cpp)
target_link_libraries(test-bounding-box boost_system)
pods_use_pkg_config_packages(test-bounding-box lcm ${PCL_IO_PKG_CONFIG} 
  pronto_vis pronto_lcm)
pods_install_executables(test-bounding-box)
