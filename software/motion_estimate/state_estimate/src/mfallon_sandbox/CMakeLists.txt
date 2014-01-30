cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(estimate)


################
set(PCL_LIBRARIES  pcl_io-1.6)
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)


add_library(estimate  SHARED leg_odometry.cpp)
pods_use_pkg_config_packages(estimate ${PCL_LIBRARIES}
  ${PKG_CONFIG_DEPS}  opencv
  lcmtypes_bot2-core lcmtypes_fovis-bot2 
  pointcloud_math  pointcloud_vis
  leg-odometry inertial-odometry 
  eigen3)
set_target_properties(estimate PROPERTIES SOVERSION 1)
pods_install_libraries(estimate)
pods_install_headers( leg_odometry.hpp common_conversions.hpp DESTINATION estimate)
pods_install_pkg_config_file(estimate
  LIBS -lestimate -lleg-odometry -linertial-odometry -lpcl_io
  REQUIRES pcl_io-1.6
  VERSION 0.0.1)


################
add_executable(se-leg-odometry main.cpp )
pods_use_pkg_config_packages(se-leg-odometry estimate path-util
   bot2-param-client model-client  pointcloud_math bot2-frames_cpp)
target_link_libraries (se-leg-odometry lcm )
#     
#        pointcloud_lcm                pointcloud_math
#	lcmtypes_drc_lcmtypes  lcmtypes_visualization
#       ${PCL_LIBRARIES} ${REQUIRED_PACKAGES}
#       leg-odometry)
pods_install_executables(se-leg-odometry )




################
add_executable(se-convert-bdi-into-vicon-frame convert_bdi_into_vicon_frame.cpp )
pods_use_pkg_config_packages(se-convert-bdi-into-vicon-frame  bot2-frames_cpp
    path-util
    pointcloud_vis  )
target_link_libraries (se-convert-bdi-into-vicon-frame lcm pointcloud_math  )
pods_install_executables(se-convert-bdi-into-vicon-frame )