set (ROS_PACKAGE_NAME sandia_hand_description)
set (ROS_PKG_INSTALL_PATH ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/ros/${ROS_PACKAGE_NAME})

set (files
  manifest.xml
)
install(FILES ${files} DESTINATION ${ROS_PKG_INSTALL_PATH})
add_subdirectory(launch)
add_subdirectory(meshes)
add_subdirectory(robots)
add_subdirectory(urdf)
add_subdirectory(gazebo)
add_subdirectory(config)
