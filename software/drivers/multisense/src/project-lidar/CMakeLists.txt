set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)



# this is not correctly calculating the lidar transforms, out of date:
add_executable(multisense-lidar-to-3dpoints lidar-to-3dpoints.cpp)
pods_use_pkg_config_packages(multisense-lidar-to-3dpoints lcm 
  bot2-lcmgl-client eigen3)
pods_install_executables(multisense-lidar-to-3dpoints)



add_executable(multisense-lidar-to-3dpoints-libbot lidar-to-3dpoints-libbot.cpp)
pods_use_pkg_config_packages(multisense-lidar-to-3dpoints-libbot lcm 
              bot2-frames bot2-lcmgl-client eigen3)
pods_install_executables(multisense-lidar-to-3dpoints-libbot)



# Functionally identical version - except with PCL point clouds
# and Maurice Fallon's renderering utilities
#add_executable(multisense-lidar-to-3dpoints-pcl lidar-to-3dpoints-pcl.cpp)
#pods_use_pkg_config_packages(multisense-lidar-to-3dpoints-pcl lcm 
#   lcmtypes_visualization pointcloud_vis)
#pods_install_executables(multisense-lidar-to-3dpoints-pcl)

# deprecated:
#add_executable(multisense-lidar-frames lidar-frames.cpp  )
#pods_use_pkg_config_packages(multisense-lidar-frames eigen3
#        lcmtypes_multisense)
#pods_install_executables(multisense-lidar-frames )

