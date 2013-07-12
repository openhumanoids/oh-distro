cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries
rosbuild_add_library(${PROJECT_NAME} src/laser.cpp
				     src/camera.cpp
				     src/state_publisher.cpp)

rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#target_link_libraries(${PROJECT_NAME} another_library)
rosbuild_add_executable(ros_driver src/ros_driver.cpp)
target_link_libraries(ros_driver ${PROJECT_NAME})
target_link_libraries(ros_driver yaml-cpp)

rosbuild_add_executable(raw_snapshot src/raw_snapshot.cpp)
target_link_libraries(raw_snapshot ${PROJECT_NAME})
