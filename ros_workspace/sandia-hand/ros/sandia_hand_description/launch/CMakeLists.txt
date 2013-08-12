set (files
  sandia_hand.launch
  sandia_hand_empty_world.launch
  upload.launch
  visualize.launch)
install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/launch)
