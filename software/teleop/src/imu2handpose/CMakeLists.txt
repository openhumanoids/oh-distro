set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

#################################################################
add_executable(drc-imu2handpose imu2handpose.cpp)
target_link_libraries(drc-imu2handpose   ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-imu2handpose lcmtypes_microstrain_comm
  lcmtypes_drc_lcmtypes  
	lcmtypes_bot2-core eigen3)
#  unpacktemp  kinect-utils jpeg_utils
#		path-util   visualization   vis_config  math_utils  kmcl_utils  kmcl_filters
#		lcmtypes_kinect  lcmtypes_kmcl  lcmtypes_microstrain_comm
#                fovision  lcmtypes_fovis-bot2 obstacle
#		lcmtypes_visualization)
pods_install_executables(drc-imu2handpose)
