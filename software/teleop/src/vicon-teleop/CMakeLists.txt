cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
#set(POD_NAME teleop)

set(REQUIRED_PACKAGES lcmtypes_vicon-drc  lcmtypes_teleop orocos-kdl kdl-parser forward-kinematics)


add_executable(drc-vicon-teleop vicon-teleop.cpp)
target_link_libraries(drc-vicon-teleop lcm )
pods_use_pkg_config_packages(drc-vicon-teleop lcm
  bot2-core bot2-vis bot2-frames
  drc-utils
  lcmtypes_drc_lcmtypes  ${REQUIRED_PACKAGES} bot2-lcmgl-client
  pointcloud_vis 
  )    
pods_install_executables(drc-vicon-teleop)


