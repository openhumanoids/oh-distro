set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(registration)

##############################################

add_executable (doMatching2D doMatching2D.cpp)
target_link_libraries(doMatching2D boost_system)
pods_use_pkg_config_packages(doMatching2D eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom icp-registration
  ${FK_PACKAGES})
pods_install_executables(doMatching2D)

add_executable (icp-error-compute icp_error_compute.cpp)
target_link_libraries(icp-error-compute boost_system)
pods_use_pkg_config_packages(icp-error-compute eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
  pronto_vis icp-registration
  ${FK_PACKAGES})
pods_install_executables(icp-error-compute)



