set (files
  sandia_hand.gazebo.xacro
  sandia_hand.transmission.xacro
  sandia_hand.urdf.xacro
  sandia_hand_simple_shapes.urdf.xacro
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/ros/${ROS_PACKAGE_NAME}/urdf/)

