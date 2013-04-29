find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

#add_definitions( -std=gnu99)

#####################################################################
add_library(renderer_robot_state SHARED 
  renderer_robot_state.cpp
  RobotStateListener.cpp
  )
  
set_target_properties(renderer_robot_state  PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_robot_state )
pods_install_headers(renderer_robot_state.hpp
  RobotStateListener.hpp
  DESTINATION renderer_robot_state )


# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_ROBOT_STATE)
    message("REQUIRED_PACKAGES_ROBOT_STATE already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_ROBOT_STATE)

set(REQUIRED_PACKAGES_ROBOT_STATE lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector collision_object visualization-utils eigen3 ) 

pods_install_pkg_config_file(renderer_robot_state 
    LIBS -lrenderer_robot_state  -ljpeg 
    REQUIRES ${REQUIRED_PACKAGES_ROBOT_STATE} 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_robot_state 
  ${REQUIRED_PACKAGES_ROBOT_STATE})
