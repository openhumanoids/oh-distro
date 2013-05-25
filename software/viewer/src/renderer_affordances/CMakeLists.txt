find_package(PkgConfig REQUIRED)
#pkg_check_modules(BOT2_VIS bot2-vis)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer")
#    return()
#endif(NOT BOT2_VIS_FOUND)

#add_definitions( -std=gnu99)

# required for affordance-utils
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

set(PCL_LIBRARIES pcl_io-1.6 pcl_filters-1.6
    pcl_segmentation-1.6 pcl_sample_consensus-1.6
    pcl_features-1.6)

#####################################################################
add_library(renderer_affordances SHARED 
  renderer_affordances.cpp
  RobotStateListener.cpp
  AffordanceCollectionListener.cpp
  CandidateGraspSeedListener.cpp
  InitGraspOptPublisher.cpp
  GraspOptStatusListener.cpp
  CandidateFootStepSeedManager.cpp
  ReachabilityVerifier.cpp
  BatchFKQueryHandler.cpp
  object_interaction_gui_utils.cpp
  )
  
set_target_properties(renderer_affordances  PROPERTIES SOVERSION 1)

pods_install_libraries(renderer_affordances )
pods_install_headers(renderer_affordances.hpp
  RobotStateListener.hpp
  AffordanceCollectionListener.hpp
  CandidateGraspSeedListener.hpp
  InitGraspOptPublisher.hpp
  GraspOptStatusListener.hpp
  CandidateFootStepSeedManager.hpp
  ReachabilityVerifier.hpp
  BatchFKQueryHandler.hpp
  otdf_instance_management_gui_utils.hpp
  object_interaction_gui_utils.hpp
  lcm_utils.hpp
  DESTINATION renderer_affordances )


# create a pkg-config file for the library, to make it easier for other
# software to use.
if(REQUIRED_PACKAGES_AFFORDANCES)
    message("REQUIRED_PACKAGES_AFFORDANCES already defined.  Not building viewer")
    return()
endif(REQUIRED_PACKAGES_AFFORDANCES)

set(REQUIRED_PACKAGES_AFFORDANCES lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector collision_object visualization-utils eigen3 kinematics_model_gfe     ${PCL_LIBRARIES}) 

pods_install_pkg_config_file(renderer_affordances 
    LIBS -lrenderer_affordances  -ljpeg 
    REQUIRES ${REQUIRED_PACKAGES_AFFORDANCES} 
    VERSION 0.0.1)

pods_use_pkg_config_packages(renderer_affordances affordance-utils
  ${REQUIRED_PACKAGES_AFFORDANCES})
