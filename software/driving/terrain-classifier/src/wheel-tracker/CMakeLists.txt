set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)

set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)

# create an executable, and make it public
add_executable(drc-wheel-estimator car-wheel-estimation-app.cpp)
pods_use_pkg_config_packages(drc-wheel-estimator bot2-core bot2-frames opencv perception-opencv-utils image_io_utils bot2-lcmgl-client occ-map bot2-frames_cpp stereo-bm histogram-tracker ${PCL_LIBRARIES}) 
pods_install_executables(drc-wheel-estimator)
