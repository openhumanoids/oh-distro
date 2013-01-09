cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME visualization_utils)
include(cmake/pods.cmake)


FILE(GLOB INTERACTABLE_KINEMATIC_BODY_HDRS src/[^.]*.hpp)
pods_install_headers(${INTERACTABLE_KINEMATIC_BODY_HDRS} DESTINATION visualization_utils)
               
# create an executable, and make it public
add_library(visualization-utils SHARED  src/GlKinematicBody.cpp src/InteractableGlKinematicBody.cpp) # 
  
# make the library public
pods_install_libraries(visualization-utils)

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detection eigen3) 
pods_use_pkg_config_packages(visualization-utils lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(visualization-utils
    CFLAGS
    LIBS -lvisualization-utils
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
