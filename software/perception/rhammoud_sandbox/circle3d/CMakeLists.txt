cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(circle3d)

#set(PCL_DIR "/home/hammoud/drc/software/externals/pcl_drc/pcl-1.6.0")
set(PCL_DIR "/home/hammoud/drc/software/perception/rhammoud_sandbox/pcl_trunk1.7/trunk/")
find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (circle3d circle3d.cpp)
target_link_libraries (circle3d ${PCL_LIBRARIES})
 

