set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
add_executable (object-tracker object-tracker.cpp )
target_link_libraries(object-tracker particle)
pods_use_pkg_config_packages(object-tracker   particle
  gsl  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  bot2-param-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_math opencv image_io_utils)
pods_install_executables(object-tracker)




add_executable (color-tracker color_tracker.cpp )
target_link_libraries(color-tracker particle)
pods_use_pkg_config_packages(color-tracker   particle
  gsl  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  pointcloud_vis pointcloud_math opencv image_io_utils)
pods_install_executables(color-tracker)
