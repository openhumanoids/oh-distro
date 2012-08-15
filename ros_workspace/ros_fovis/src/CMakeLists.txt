cmake_minimum_required(VERSION 2.4.6)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(TRUE_robot_state_publisher TRUE_robot_state_publisher.cpp)
pods_use_pkg_config_packages(TRUE_robot_state_publisher lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(ros_fovis ros_fovis.cpp)
target_link_libraries(ros_fovis fovis)
pods_use_pkg_config_packages(ros2lcm_translator lcm lcmtypes_drc_lcmtypes
	bot2-core  lcmtypes_bot2-core lcmtypes_drc_lcmtypes lcmtypes_visualization) 


# Old Stuff:
#cmake_minimum_required(VERSION 2.4.6)
#include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
#rosbuild_init()
##set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
##set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
#add_definitions(-Wall -O3 -mfpmath=sse -mmmx -msse -msse2 -msse3)
#include_directories(${PROJECT_SOURCE_DIR}/build/include)
#link_directories(${PROJECT_SOURCE_DIR}/build/lib)
#rosbuild_add_executable(vo_fovis src/vo_fovis.cpp)
#target_link_libraries(vo_fovis fovis)
