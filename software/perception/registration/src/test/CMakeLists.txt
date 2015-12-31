##############################################
add_executable (testRegister2D testRegister2D.cpp)
target_link_libraries(testRegister2D boost_system)
pods_use_pkg_config_packages(testRegister2D eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom
  ${FK_PACKAGES})
pods_install_executables(testRegister2D)


