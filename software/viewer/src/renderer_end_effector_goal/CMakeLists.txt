find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_end_effector_goal SHARED 
  renderer_end_effector_goal.cpp
  RobotStateListener.cpp
  )
  
set_target_properties(renderer_end_effector_goal  PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_end_effector_goal )
pods_install_headers(renderer_end_effector_goal.hpp
  RobotStateListener.hpp
  DESTINATION renderer_end_effector_goal )


# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_EE_GOAL)
    message("REQUIRED_PACKAGES_EE_GOAL already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_EE_GOAL)

set(REQUIRED_PACKAGES_EE_GOAL lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector visualization-utils eigen3 ) 

pods_install_pkg_config_file(renderer_end_effector_goal
    LIBS -lrenderer_end_effector_goal  -ljpeg 
    REQUIRES ${REQUIRED_PACKAGES_EE_GOAL} 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_end_effector_goal
  ${REQUIRED_PACKAGES_EE_GOAL})
