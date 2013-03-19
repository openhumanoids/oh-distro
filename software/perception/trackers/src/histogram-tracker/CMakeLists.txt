set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")



add_library(tracker-histogram  SHARED HistTracker.cpp)
pods_use_pkg_config_packages(tracker-histogram 
   image_io_utils
   eigen3 opencv)
target_link_libraries (tracker-histogram boost_system 
           ${REQUIRED_LIBS})
set_target_properties(tracker-histogram PROPERTIES SOVERSION 1)
pods_install_libraries(tracker-histogram)
pods_install_headers( HistTracker.hpp DESTINATION trackers)
pods_install_pkg_config_file(tracker-histogram
  LIBS -ltracker-histogram
  VERSION 0.0.1)


add_executable(drc-histogram-tracker 
  histogram-tracker.cpp
  HistTracker.cpp)
pods_use_pkg_config_packages(drc-histogram-tracker lcm 
  bot2-param-client bot2-lcmgl-client
  lcmtypes_tld-tracker
  bot2-frames  bot2-frames_cpp
  opencv  image_io_utils tracker-histogram
  pointcloud_filters  pointcloud_vis pointcloud_lcm)
pods_install_executables(drc-histogram-tracker)
