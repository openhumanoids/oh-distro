cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
#set(POD_NAME teleop)

set(REQUIRED_PACKAGES lcmtypes_vicon-drc  lcmtypes_teleop orocos-kdl kdl-parser forward-kinematics)

    
add_executable(vicon_teleop_ee_paladin_v1 vicon_teleop_ee_paladin_v1.cpp)
target_link_libraries(vicon_teleop_ee_paladin_v1 lcm)
pods_use_pkg_config_packages(vicon_teleop_ee_paladin_v1 lcm lcmtypes_vicon-drc lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES} bot2-lcmgl-client)    
pods_install_executables(vicon_teleop_ee_paladin_v1)


