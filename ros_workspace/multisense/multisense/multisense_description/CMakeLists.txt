set (ROS_PACKAGE_NAME multisense_sl_description)
set (ROS_PKG_INSTALL_PATH share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME})

set (files
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${DRC_SIM_VERSION}/ros/${ROS_PACKAGE_NAME}/)
add_subdirectory(robots)
add_subdirectory(urdf)
add_subdirectory(gazebo)
