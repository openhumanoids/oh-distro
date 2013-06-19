find_package(PkgConfig REQUIRED)
add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

#####################################################################
add_library(renderer_data_control SHARED 
  DataControlRenderer.cpp)
  
set_target_properties(renderer_data_control PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_data_control affordance
  drc-utils lcmtypes_drc_lcmtypes gtkmm-renderer gtkmm-2.4)



pods_install_headers(DataControlRenderer.hpp
  DESTINATION renderer_data_control)

pods_install_libraries(renderer_data_control)
  
pods_install_pkg_config_file(renderer_data_control
    LIBS -lrenderer_data_control
    REQUIRES lcmtypes_drc_lcmtypes drc-utils gtkmm-renderer
    VERSION 0.0.1)


#####################################################################
set(PACKAGE_NAME renderer_annotated_camera)

add_library(${PACKAGE_NAME} SHARED AnnotatedCameraRenderer.cpp)
set_target_properties(${PACKAGE_NAME} PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(${PACKAGE_NAME} affordance
  drc-utils lcmtypes_drc_lcmtypes gtkmm-renderer gtkmm-2.4 bot2-lcmgl-client)

pods_install_headers(AnnotatedCameraRenderer.hpp DESTINATION ${PACKAGE_NAME})
pods_install_libraries(${PACKAGE_NAME})
  
pods_install_pkg_config_file(${PACKAGE_NAME}
    LIBS -l${PACKAGE_NAME}
    REQUIRES lcmtypes_drc_lcmtypes drc-utils bot2-lcmgl-client gtkmm-renderer
    VERSION 0.0.1)


#####################################################################
add_library(renderer_maps SHARED 
  MapsRenderer.cpp
  MeshRenderer.cpp
  InteractiveBox.cpp
)
  
set_target_properties(renderer_maps PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_maps
  maps jpeg-utils occ-map gtkmm-renderer gtkmm-2.4)

pods_install_headers(
  MapsRenderer.hpp
  DESTINATION renderer_maps)

pods_install_libraries(renderer_maps)
  
pods_install_pkg_config_file(renderer_maps
    LIBS -lrenderer_maps
    REQUIRES bot2-vis maps
    VERSION 0.0.1)


#####################################################################
add_library(renderer_lidartest SHARED 
  MeshRenderer.cpp
  LidarTestRenderer.cpp
)

set_target_properties(renderer_lidartest PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(renderer_lidartest
  lcmtypes_drc_lcmtypes eigen3
  bot2-frames jpeg-utils maps gtkmm-renderer gtkmm-2.4)

pods_install_headers(LidarTestRenderer.hpp
  DESTINATION renderer_lidartest)

pods_install_libraries(renderer_lidartest)
  
pods_install_pkg_config_file(renderer_lidartest
    LIBS -lrenderer_lidartest
    REQUIRES bot2-frames gtkmm-renderer gtkmm-2.4
    VERSION 0.0.1)




set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
add_executable(drc-annotation-viewer AnnotationViewer.cpp udp_util.c RendererGroupUtil.cpp)
pods_use_pkg_config_packages(drc-annotation-viewer 
	bot2-vis  
        bot2-lcmgl-renderer
        bot2-frames-renderers  
        laser-util-renderer
        image-util-renderer
        renderer_walking
        multisense-renderer
        octomap-renderer
	occ-map-renderers
        renderer_maps
        lcmtypes_drc_lcmtypes
        renderer_robot_state
        renderer_robot_plan
        renderer_crawling_plan
        renderer_affordances
        renderer_sticky_feet
        renderer_end_effector_goal
        #renderer_drc 
        collections_renderer
        renderer_manipulation
        renderer_driving
        #renderer_walking
        renderer_scrollingplots
        renderer_status
        renderer_data_control
        renderer_annotated_camera
        renderer_drcgrid
        tracker-renderer
        renderer_recovery)
#scrollingplots_renderer
pods_install_executables(drc-annotation-viewer)
