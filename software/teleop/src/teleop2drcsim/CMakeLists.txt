# pull in the pods macros. See cmake/pods.cmake for documentation
#set(POD_NAME teleop)

# Check this
set(REQUIRED_PACKAGES lcmtypes_teleop urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(vicon_eegoals_publisher vicon_eegoals_publisher.cpp)
target_link_libraries(vicon_eegoals_publisher lcm)
pods_use_pkg_config_packages(vicon_eegoals_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})


pods_install_executables(vicon_eegoals_publisher)
