set (files 
  base_link.STL
  hokuyo_link.STL
)
install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/multisense_sl/meshes/)
