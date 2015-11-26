set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-lidar-passthrough lidar-passthrough.cpp)
target_link_libraries(drc-lidar-passthrough boost_system)
pods_use_pkg_config_packages(drc-lidar-passthrough lcm 
  model-client
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pronto_vis pronto_lcm drake-rbm drake-geometry-util)
pods_install_executables(drc-lidar-passthrough)

# Simple scan-by-scan filtering:
add_executable(drc-lidar-filter lidar-filter.cpp)
target_link_libraries(drc-lidar-filter boost_system)
pods_use_pkg_config_packages(drc-lidar-filter lcm
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp)
pods_install_executables(drc-lidar-filter)




