set (files 
  base_link.stl
  hokuyo_link.stl
  head.dae
  head_camera.dae
)
install(FILES ${files} DESTINATION ${GAZEBO_INSTALL_PATH}/multisense_sl_description/meshes/)
