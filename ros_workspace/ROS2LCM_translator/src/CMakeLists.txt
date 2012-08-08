cmake_minimum_required(VERSION 2.4.6)

rosbuild_add_executable(TRUE_robot_state_publisher TRUE_robot_state_publisher.cpp)
pods_use_pkg_config_packages(TRUE_robot_state_publisher lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(ros2lcm_translator ros2lcm_translator.cpp)
pods_use_pkg_config_packages(ros2lcm_translator lcm lcmtypes_drc_lcmtypes bot2-core  lcmtypes_bot2-core) 


pods_install_executables(TRUE_robot_state_publisher  ros2lcm_translator)

# Old Stuff:
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#pods_install_executables(TRUE_robot_state_publisher)
#pods_use_pkg_config_packages(TRUE_robot_state_publisher lcm #lcmtypes_drc_lcmtypes bot2-core) 

