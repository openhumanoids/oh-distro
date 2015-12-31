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
# Executables for testing - copied from lib tutorial - TODO remove
add_executable (testing_colortree test_color_tree.cpp)
target_link_libraries(testing_colortree boost_system
                       boost_thread octomap)
pods_use_pkg_config_packages(testing_colortree eigen3
  bot2-frames
  path-util
  ${REQUIRED_PACKAGES} pcl_io-1.7
  pronto_vis
  octomap-util
  ${FK_PACKAGES})
pods_install_executables(testing_colortree)



