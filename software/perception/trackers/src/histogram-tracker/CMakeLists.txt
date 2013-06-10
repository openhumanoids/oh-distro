set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")



add_library(histogram-tracker  SHARED histogram-tracker.cpp)
pods_use_pkg_config_packages(histogram-tracker 
   image_io_utils
   eigen3 opencv)
target_link_libraries (histogram-tracker boost_system 
           ${REQUIRED_LIBS} ${LCMTYPES_LIBS})
set_target_properties(histogram-tracker PROPERTIES SOVERSION 1)
pods_install_libraries(histogram-tracker)
pods_install_headers( histogram-tracker.hpp DESTINATION trackers)
pods_install_pkg_config_file(histogram-tracker
  LIBS -lhistogram-tracker
  VERSION 0.0.1)


add_executable(drc-histogram-tracker 
  histogram-tracker-app.cpp
  histogram-tracker.cpp)
pods_use_pkg_config_packages(drc-histogram-tracker lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  opencv  image_io_utils histogram-tracker
  pointcloud_filters  pointcloud_vis pointcloud_lcm)
target_link_libraries (drc-histogram-tracker boost_system 
           ${REQUIRED_LIBS} ${LCMTYPES_LIBS})
pods_install_executables(drc-histogram-tracker)


add_library(object-segmenter  SHARED object-segmenter.cpp)
pods_use_pkg_config_packages(object-segmenter histogram-tracker
   image_io_utils
   eigen3 opencv)
target_link_libraries (object-segmenter boost_system 
           ${REQUIRED_LIBS} ${LCMTYPES_LIBS})
set_target_properties(object-segmenter PROPERTIES SOVERSION 1)
pods_install_libraries(object-segmenter)
pods_install_headers( object-segmenter.hpp DESTINATION trackers)
pods_install_pkg_config_file(object-segmenter
  LIBS -lobject-segmenter
  VERSION 0.0.1)


add_executable(drc-object-segmentation 
  object-segmentation-app.cpp)
pods_use_pkg_config_packages(drc-object-segmentation lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  opencv image_io_utils pointcloud_filters  pointcloud_vis pointcloud_lcm histogram-tracker object-segmenter lcmtypes_drc_lcmtypes)
target_link_libraries (drc-object-segmentation boost_system 
           ${REQUIRED_LIBS} ${LCMTYPES_LIBS})
pods_install_executables(drc-object-segmentation)


add_executable(drc-object-annotater 
  object-annotater-app.cpp)
pods_use_pkg_config_packages(drc-object-annotater lcm 
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp
  opencv  image_io_utils pointcloud_filters  pointcloud_vis pointcloud_lcm histogram-tracker object-segmenter lcmtypes_drc_lcmtypes)
target_link_libraries (drc-object-annotater boost_system 
           ${REQUIRED_LIBS} ${LCMTYPES_LIBS})
pods_install_executables(drc-object-annotater)
