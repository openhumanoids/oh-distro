set (files 
  base_link.stl
  hokuyo_link.stl
)
install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/multisense_sl/meshes/)
