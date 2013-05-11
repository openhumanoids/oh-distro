set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

include(cmake/pods.cmake)
set(POD_NAME translators)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_executable(lcm2ros src/lcm2ros.cpp)
pods_install_executables(lcm2ros)
pods_use_pkg_config_packages(lcm2ros lcm lcmtypes_drc_lcmtypes bot2-core) 


rosbuild_add_executable(ros2lcm src/ros2lcm.cpp)
target_link_libraries(ros2lcm boost_signals)
pods_install_executables(ros2lcm)
pods_use_pkg_config_packages(ros2lcm lcm lcmtypes_drc_lcmtypes bot2-core lcmtypes_multisense) 


rosbuild_add_executable(latency src/latency.cpp)
target_link_libraries(latency boost_signals)
pods_install_executables(latency)
pods_use_pkg_config_packages(latency latency) 

