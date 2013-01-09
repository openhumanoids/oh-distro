find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_robot_plan SHARED 
  renderer_robot_plan.cpp
  RobotPlanListener.cpp
  )
  
set_target_properties(renderer_robot_plan  PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_robot_plan )
pods_install_headers(renderer_robot_plan.hpp
  RobotPlanListener.hpp
  DESTINATION renderer_robot_plan )


# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_ROBOT_PLAN)
    message("REQUIRED_PACKAGES_ROBOT_PLAN already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_ROBOT_PLAN)

set(REQUIRED_PACKAGES_ROBOT_PLAN lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detection visualization-utils eigen3 ) 

pods_install_pkg_config_file(renderer_robot_plan 
    LIBS -lrenderer_robot_plan  -ljpeg 
    REQUIRES ${REQUIRED_PACKAGES_ROBOT_PLAN} 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_robot_plan 
  ${REQUIRED_PACKAGES_ROBOT_PLAN})
