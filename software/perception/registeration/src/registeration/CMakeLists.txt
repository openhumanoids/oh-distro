################
set ( OpenCV_LIBS opencv_core
                  opencv_highgui
                  opencv_nonfree
                opencv_calib3d
                opencv_imgproc
                          opencv_features2d )

add_library(registeration  SHARED registeration.cpp)
pods_use_pkg_config_packages(registeration
  opencv
  lcmtypes_bot2-core bot2-lcmgl-client
  pointcloud_vis 
  eigen3 estimate-pose)
target_link_libraries (registeration ${OpenCV_LIBS})
set_target_properties(registeration PROPERTIES SOVERSION 1)
pods_install_libraries(registeration)
pods_install_headers( registeration.hpp DESTINATION registeration)
pods_install_pkg_config_file(registeration
  LIBS -lregisteration 
  VERSION 0.0.1)



################
add_executable(drc-registeration registeration-app.cpp)
target_link_libraries(drc-registeration)
pods_use_pkg_config_packages(drc-registeration
  bot2-vis visualization-utils
  pointcloud_vis opencv
  lcm bot2-lcmgl-client registeration  )
pods_install_executables(drc-registeration)


################
add_executable(drc-registeration-batch registeration-batch.cpp)
target_link_libraries(drc-registeration-batch)
pods_use_pkg_config_packages(drc-registeration-batch
  bot2-vis visualization-utils
  pointcloud_vis opencv
  lcm bot2-lcmgl-client registeration  )
pods_install_executables(drc-registeration-batch)