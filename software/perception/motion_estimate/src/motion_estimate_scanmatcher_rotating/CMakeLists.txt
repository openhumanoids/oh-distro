include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${LCMTYPES_INCLUDE_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-motion-estimate-horzscan scan_matcher_horizontal.cpp)
pods_use_pkg_config_packages(drc-motion-estimate-horzscan collision_object_gfe collision_detector collision_object lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_lcm scanmatch)
pods_install_executables(drc-motion-estimate-horzscan)
