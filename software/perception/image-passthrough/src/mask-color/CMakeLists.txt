set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-get-lidar-depth-image get-lidar-depth-image.cpp)
pods_use_pkg_config_packages(drc-get-lidar-depth-image maps bot2-lcmgl-client
              pointcloud_vis bot2-frames bot2-frames_cpp
              lcmtypes_multisense
              image_io_utils)
pods_install_executables(drc-get-lidar-depth-image)

