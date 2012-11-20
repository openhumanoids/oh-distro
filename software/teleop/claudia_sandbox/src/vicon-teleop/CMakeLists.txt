cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
#set(POD_NAME vicon-teleop)

set(REQUIRED_PACKAGES lcmtypes_vicon  lcmtypes_teleop orocos-kdl kdl-parser forward-kinematics)

# prog
add_executable(runteleop_4markers runteleop_4markers.c)
target_link_libraries(runteleop_4markers lcm)
pods_use_pkg_config_packages(runteleop_4markers lcm lcmtypes_vicon-drc lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES}
    bot2-lcmgl-client )

# make exeutables public
pods_install_executables(runteleop_4markers)

