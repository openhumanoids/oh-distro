cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME stickyhand_utils)
include(cmake/pods.cmake)

FILE(GLOB STICKYHAND_UTIL_HDRS src/[^.]*.hpp)
pods_install_headers(${STICKYHAND_UTIL_HDRS} DESTINATION visualization_utils/stickyhand_utils)
               
# create an executable, and make it public
add_library(stickyhand-viz-utils SHARED  src/StickyhandCollectionManager.cpp) # 

# make the library public
pods_install_libraries(stickyhand-viz-utils)

set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector collision_object eigen3 visualization-utils affordance-viz-utils) 
pods_use_pkg_config_packages(stickyhand-viz-utils lcm lcmtypes_drc_lcmtypes bot2-core bot2-vis bot2-frames ${REQUIRED_PACKAGES})

# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(stickyhand-viz-utils
    CFLAGS
    LIBS -lstickyhand-viz-utils bot2-vis
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
