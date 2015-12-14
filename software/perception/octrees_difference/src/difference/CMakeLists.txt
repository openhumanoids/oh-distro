set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(octrees_difference)

##############################################
add_executable (icp-octrees-diff octrees_diff_app.cpp convert_octomap.cpp)
target_link_libraries(icp-octrees-diff pointmatcher lcm boost_system
                       boost_thread pronto_math)
pods_use_pkg_config_packages(icp-octrees-diff eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
  pronto_vis icp-registration
  octomap-util
  ${FK_PACKAGES})
pods_install_executables(icp-octrees-diff)

##################
add_executable (test_icp_changedkeys test_changedkeys.cpp)
target_link_libraries(test_icp_changedkeys boost_system
                       boost_thread octomap)
pods_use_pkg_config_packages(test_icp_changedkeys eigen3
  lcm    bot2-frames
  path-util
  lcmtypes_bot2-core octomap
  lidar-odom ${REQUIRED_PACKAGES} pcl_io-1.7
  pronto_vis icp-registration
  octomap-util
  ${FK_PACKAGES})
pods_install_executables(test_icp_changedkeys)


