cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(estimate_tools)


################


add_library(estimate_tools  SHARED imu_stream.cpp iir_notch.cpp)
pods_use_pkg_config_packages(estimate_tools  ${PKG_CONFIG_DEPS}  
  eigen3)
set_target_properties(estimate_tools PROPERTIES SOVERSION 1)
pods_install_libraries(estimate_tools)
pods_install_headers( imu_stream.hpp iir_notch.hpp
              DESTINATION estimate_tools)
pods_install_pkg_config_file(estimate_tools
  LIBS -lestimate_tools
  REQUIRES 
  VERSION 0.0.1)



################
add_executable(se-filtering-demo filtering_demo.cpp )
pods_use_pkg_config_packages(se-filtering-demo bot2-frames_cpp
    path-util filter_tools estimate_tools
    pointcloud_vis  )
target_link_libraries (se-filtering-demo lcm pointcloud_math  )
pods_install_executables(se-filtering-demo )
