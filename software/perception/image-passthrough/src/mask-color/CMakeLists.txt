set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-mask-color-lidar mask-color-lidar.cpp)
pods_use_pkg_config_packages(drc-mask-color-lidar maps bot2-lcmgl-client
              pointcloud_vis bot2-frames bot2-frames_cpp
              image_io_utils)
pods_install_executables(drc-mask-color-lidar)

