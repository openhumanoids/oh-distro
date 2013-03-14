add_executable(drc-plane-detector plane_detector.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(drc-plane-detector maps
   pointcloud_vis pointcloud_filters)
pods_install_executables(drc-plane-detector)



add_executable(drc-plane-fitter plane_fitter.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(drc-plane-fitter pointcloud_vis pointcloud_filters)
pods_install_executables(drc-plane-fitter)




add_executable(drc-auto-octomap-reset auto_octomap_reset.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(drc-auto-octomap-reset pointcloud_vis pointcloud_filters)
pods_install_executables(drc-auto-octomap-reset)