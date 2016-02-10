##############################################
add_executable (testRegister2D testRegister2D.cpp)
target_link_libraries(testRegister2D boost_system)
pods_use_pkg_config_packages(testRegister2D eigen3
  lcm    bot2-frames
  lcmtypes_bot2-core  
  lidar-odom
  ${FK_PACKAGES})
pods_install_executables(testRegister2D)

###############
add_executable(color-octomap-server color-octomap-server.cpp)
pods_use_pkg_config_packages(color-octomap-server octomap-util)
pods_install_executables(color-octomap-server)

###############
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
add_executable(convert-vtk-to-color-octree convert-vtk-to-color-octree.cpp)
pods_use_pkg_config_packages(convert-vtk-to-color-octree path-util ${REQUIRED_PACKAGES} pcl_io-1.7
    pronto_vis eigen3 bot2-frames octomap octomap-util)
target_link_libraries(convert-vtk-to-color-octree  ${VTK_LIBRARIES})
target_link_libraries(convert-vtk-to-color-octree icp-registration lcm pronto_math  
                        boost_system boost_thread)
pods_install_executables(convert-vtk-to-color-octree)
