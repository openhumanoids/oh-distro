#
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
# CONVENTION IS FOR RENDERER FILES TO BE NAMED
# renderer_name.cpp and renderer_name.hpp
# NBNBNBNBNBNBNNBNBNBNBNBNBNBNBNNBNB
#

find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_drc SHARED 
  end_effector_goal_renderer/renderer_end_effector_goal.cpp
  end_effector_goal_renderer/RobotStateListener.cpp

  )
  
set_target_properties(renderer_drc PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_drc)

  
pods_install_headers(end_effector_goal_renderer/renderer_end_effector_goal.hpp
end_effector_goal_renderer/RobotStateListener.hpp
  DESTINATION renderer_drc/end_effector_goal_renderer)


# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_HUMANOID)
    message("REQUIRED_PACKAGES_HUMANOID already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_HUMANOID)

set(REQUIRED_PACKAGES_HUMANOID urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detection eigen3) 

pods_install_pkg_config_file(renderer_drc
    LIBS -lrenderer_drc -ljpeg 
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_drc
  lcmtypes_drc_lcmtypes
  bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES_HUMANOID})
