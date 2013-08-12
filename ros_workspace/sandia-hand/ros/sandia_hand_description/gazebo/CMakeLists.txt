set (files
  database.config
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/ros/${ROS_PACKAGE_NAME}/gazebo/)

