# this is not correctly calculating the lidar transforms, out of date:
add_executable(mf-fisheye fisheye.cpp)
pods_use_pkg_config_packages(mf-fisheye lcm image_io_utils
              bot2-frames bot2-lcmgl-client eigen3)
pods_install_executables(mf-fisheye)
