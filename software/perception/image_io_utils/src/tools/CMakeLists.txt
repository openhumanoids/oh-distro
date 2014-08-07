set(PCL_LIBRARIES  pcl_io-1.6)

add_executable(drc-multisense-image-tool multisense-image-tool.cpp  )
pods_use_pkg_config_packages(drc-multisense-image-tool
  lcmtypes_bot2-core lcmtypes_multisense
  ${PCL_LIBRARIES} pointcloud_vis
  multisense_utils   bot2-core
  bot2-frames  bot2-frames_cpp
  image_io_utils  camera_params  opencv)
target_link_libraries(drc-multisense-image-tool boost_system)
pods_install_executables(drc-multisense-image-tool )


add_executable(drc-multisense-image-tool-basic multisense-image-tool-basic.cpp  )
pods_use_pkg_config_packages(drc-multisense-image-tool-basic lcm
  lcmtypes_bot2-core lcmtypes_multisense
  ${PCL_LIBRARIES} pointcloud_vis
  multisense_utils opencv )
pods_install_executables(drc-multisense-image-tool-basic )


add_executable(drc-video-write video-write.cpp  )
pods_use_pkg_config_packages(drc-video-write
  opencv)
pods_install_executables(drc-video-write )



add_executable(drc-camera-capture capture-camera.cpp)
pods_use_pkg_config_packages(drc-camera-capture
  opencv)
pods_install_executables(drc-camera-capture)

add_executable(lcm-log-to-avi log_to_avi.cpp)
pods_use_pkg_config_packages(lcm-log-to-avi lcm 
  image_io_utils
  opencv)
pods_install_executables(lcm-log-to-avi)

#add_executable(old-robot-state-upgrade old-robot-state-upgrade.cpp  )
#pods_use_pkg_config_packages(old-robot-state-upgrade
#  lcmtypes_multisense)
#pods_install_executables(old-robot-state-upgrade )
