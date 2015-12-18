add_subdirectory(maps-renderer)
add_subdirectory(maps-viewer)

# Last compiled in 2014:
#add_subdirectory(data-control-renderer)
#add_subdirectory(camera-renderer)

# annotated camera renderer ###############################
# set(PACKAGE_NAME renderer_annotated_camera)

# add_library(${PACKAGE_NAME} SHARED AnnotatedCameraRenderer.cpp)
# set_target_properties(${PACKAGE_NAME} PROPERTIES SOVERSION 1)

# pods_use_pkg_config_packages(${PACKAGE_NAME} affordance
#   drc-utils lcmtypes_drc_lcmtypes gtkmm-renderer gtkmm-2.4 bot2-lcmgl-client)

# pods_install_headers(AnnotatedCameraRenderer.hpp DESTINATION ${PACKAGE_NAME})
# pods_install_libraries(${PACKAGE_NAME})
  
# pods_install_pkg_config_file(${PACKAGE_NAME}
#     LIBS -l${PACKAGE_NAME}
#     REQUIRES lcmtypes_drc_lcmtypes drc-utils bot2-lcmgl-client gtkmm-renderer
#     VERSION 0.0.1)


## lidar test renderer #########################################
# add_library(renderer_lidartest SHARED 
#   MeshRenderer.cpp
#   LidarTestRenderer.cpp
# )

# set_target_properties(renderer_lidartest PROPERTIES SOVERSION 1)

# pods_use_pkg_config_packages(renderer_lidartest
#   lcmtypes_drc_lcmtypes eigen3
#   bot2-frames jpeg-utils maps gtkmm-renderer gtkmm-2.4)

# pods_install_headers(LidarTestRenderer.hpp
#   DESTINATION renderer_lidartest)

# pods_install_libraries(renderer_lidartest)
  
# pods_install_pkg_config_file(renderer_lidartest
#     LIBS -lrenderer_lidartest
#     REQUIRES bot2-frames gtkmm-renderer gtkmm-2.4
#     VERSION 0.0.1)
