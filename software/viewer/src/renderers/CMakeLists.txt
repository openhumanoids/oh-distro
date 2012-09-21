find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

add_definitions(-Wall -std=gnu99)

#####################################################################
add_library(renderer_drc SHARED 
  humanoid_renderer/renderer_humanoid.cpp
  humanoid_renderer/RobotStateListener.cpp
  renderer_navigation.cpp  
  robot_plan_renderer/renderer_robot_plan.cpp
  robot_plan_renderer/RobotPlanListener.cpp
  end_effector_goal_renderer/renderer_end_effector_goal.cpp
  end_effector_goal_renderer/RobotStateListener.cpp
  otdf_renderer/renderer_otdf.cpp
  )
  
set_target_properties(renderer_drc PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_drc)
pods_install_headers(humanoid_renderer/renderer_humanoid.hpp
  humanoid_renderer/RobotStateListener.hpp
  renderer_navigation.hpp
  robot_plan_renderer/renderer_robot_plan.hpp
  robot_plan_renderer/RobotPlanListener.hpp
  DESTINATION renderer_drc)
  
pods_install_headers(end_effector_goal_renderer/renderer_end_effector_goal.hpp
end_effector_goal_renderer/RobotStateListener.hpp
  DESTINATION renderer_drc/end_effector_goal_renderer)

pods_install_headers(otdf_renderer/renderer_otdf.hpp
  DESTINATION renderer_drc/otdf_renderer)

# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_HUMANOID)
    message("REQUIRED_PACKAGES_HUMANOID already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_HUMANOID)

set(REQUIRED_PACKAGES_HUMANOID urdf orocos-kdl kdl-parser forward-kinematics otdf_dom)


pods_install_pkg_config_file(renderer_drc
    LIBS -lrenderer_drc -ljpeg 
    REQUIRES bot2-core bot2-vis bot2-frames 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_drc
  lcmtypes_drc_lcmtypes
  bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES_HUMANOID})





#####################################################################
#add_library(panorama_renderer SHARED panorama_renderer.cpp)
#pods_use_pkg_config_packages(panorama_renderer lcm bot2-vis)
#set_target_properties(panorama_renderer PROPERTIES SOVERSION 1)
#pods_install_libraries(panorama_renderer)
##pods_install_headers(panorama_renderer.hpp DESTINATION visualization)
#pods_install_pkg_config_file(panorama_renderer
#    LIBS -lpanorama_renderer
#    REQUIRES bot2-vis bot2-vis bot2-frames lcm
#    VERSION 0.0.1)
