set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(leg_estimate)


################
set(REQUIRED_PACKAGES  kdl-parser forward-kinematics)

add_library(leg_estimate  SHARED leg_estimate.cpp foot_contact_classify.cpp)
pods_use_pkg_config_packages(leg_estimate estimate_tools  filter_tools foot_contact 
  foot_contact_alt
  ${REQUIRED_PACKAGES} 
  lcmtypes_bot2-core lcmtypes_fovis-bot2 
  pointcloud_math  pointcloud_vis 
  eigen3)
set_target_properties(leg_estimate PROPERTIES SOVERSION 1)
pods_install_libraries(leg_estimate)
pods_install_headers( leg_estimate.hpp foot_contact_classify.hpp common_conversions.hpp
               DESTINATION leg_estimate)
pods_install_pkg_config_file(leg_estimate
  LIBS -lleg_estimate  -lpcl_io  -lfilter_tools -lkalman_filter_tools
  REQUIRES pcl_io-1.6
  VERSION 0.0.1)


################
add_executable(se-leg-odometry main.cpp LegOdoWrapper.cpp)
pods_use_pkg_config_packages(se-leg-odometry leg_estimate path-util
   bot2-param-client model-client pointcloud_math bot2-frames_cpp
   )
target_link_libraries (se-leg-odometry lcm )
pods_install_executables(se-leg-odometry )




################
add_executable(se-convert-bdi-into-vicon-frame convert_bdi_into_vicon_frame.cpp )
pods_use_pkg_config_packages(se-convert-bdi-into-vicon-frame  bot2-frames_cpp
    path-util ${REQUIRED_PACKAGES}
    pointcloud_vis  )
target_link_libraries (se-convert-bdi-into-vicon-frame lcm pointcloud_math  )
pods_install_executables(se-convert-bdi-into-vicon-frame )






################
add_executable(se-init-from-bdi init_from_bdi.cpp )
pods_use_pkg_config_packages(se-init-from-bdi  bot2-frames_cpp
    path-util ${REQUIRED_PACKAGES}
    pointcloud_vis  )
target_link_libraries (se-init-from-bdi lcm pointcloud_math  )
pods_install_executables(se-init-from-bdi )
