cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME drc_controller_manager)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
# include(cmake/lcmtypes.cmake)
# lcmtypes_build()

add_definitions(-DEIGEN2_SUPPORT)
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

# build programs

add_executable(drc_controller_manager src/drc_controller_manager.cpp)
#pods_use_pkg_config_packages(send_message lcm lcmtypes_${POD_NAME})
pods_use_pkg_config_packages(drc_controller_manager lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# make exeutables public
pods_install_executables(drc_controller_manager)


add_executable(fixed_ee_goal_publisher src/send_fixed_ee_goal.cpp)
#pods_use_pkg_config_packages(send_message lcm lcmtypes_${POD_NAME})
pods_use_pkg_config_packages(fixed_ee_goal_publisher lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# make exeutables public
pods_install_executables(fixed_ee_goal_publisher)


add_executable(ee_goal_listener src/ee_goal_listener.cpp)
#pods_use_pkg_config_packages(send_message lcm lcmtypes_${POD_NAME})
pods_use_pkg_config_packages(ee_goal_listener lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# make exeutables public
pods_install_executables(ee_goal_listener)

add_executable(nav_goal_listener src/nav_goal_listener.cpp)
#pods_use_pkg_config_packages(send_message lcm lcmtypes_${POD_NAME})
pods_use_pkg_config_packages(nav_goal_listener lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})

# make exeutables public
pods_install_executables(nav_goal_listener)
