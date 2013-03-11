find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_sticky_feet SHARED 
  renderer_sticky_feet.cpp
  FootStepPlanListener.cpp
  )
  
set_target_properties(renderer_sticky_feet PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_sticky_feet)
pods_install_headers(renderer_sticky_feet.hpp
  FootStepPlanListener.hpp
  plan_approval_gui_utils.hpp
  DESTINATION renderer_sticky_feet )


# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_STICKY_FEET)
    message("REQUIRED_PACKAGES_STICKY_FEET already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_STICKY_FEET)

set(REQUIRED_PACKAGES_STICKY_FEET lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector collision_object visualization-utils eigen3 ) 

pods_install_pkg_config_file(renderer_sticky_feet 
    LIBS -lrenderer_sticky_feet  -ljpeg 
    REQUIRES ${REQUIRED_PACKAGES_STICKY_FEET} 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_sticky_feet 
  ${REQUIRED_PACKAGES_STICKY_FEET})
