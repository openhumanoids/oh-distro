set (model_basenames
     sandia_hand_left_on_box
     sandia_hand_right_on_box
     sandia_hand_both_on_box
     sandia_hand_stumps_on_box
     )

foreach(model_basename ${model_basenames})
  # Bourne shell is not happy with cmake auto quoting for declaring variables 
  # in front of rosrun so first export and then make rosrun
  add_custom_command(
    OUTPUT ${model_basename}.urdf
      COMMAND export ROS_PACKAGE_PATH=${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH}
      COMMAND rosrun xacro xacro.py ${CMAKE_SOURCE_DIR}/ros/${ROS_PACKAGE_NAME}/robots/${model_basename}.urdf.xacro > ${model_basename}.urdf
    DEPENDS ${model_basename}.urdf.xacro)
  add_custom_target(${model_basename}_urdf ALL DEPENDS ${model_basename}.urdf)

  add_custom_command(
    OUTPUT ${model_basename}.model.config
    COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/gazebo_manifest_generator.sh ${model_basename} > ${model_basename}.model.config
    DEPENDS gazebo_manifest_generator.sh)
  add_custom_target(${model_basename}_manifest ALL DEPENDS ${model_basename}.model.config)
    
  install(FILES ${model_basename}.urdf.xacro
          DESTINATION ${ROS_PKG_INSTALL_PATH}/robots/)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.urdf
          DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/${model_basename})
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.model.config
          DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/${model_basename}
          RENAME model.config)
  # In gazebo-1.4, the model manifests were renamed from manifest.xml to model.config
  # For backwards compatibility, we will continue to install a duplicate manifest.xml
  # alongside the model.config for a reasonable period of time.
  # After a reasonable period of time (say the release of gazebo-1.5?),
  # the following install lines can be removed.
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.model.config
          DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/${model_basename}
          RENAME manifest.xml)
endforeach()

