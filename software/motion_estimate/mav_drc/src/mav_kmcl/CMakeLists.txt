# create the kixie state estimation executable, and make it public
add_executable(mav-kmcl mav_kmcl.cpp)
pods_use_pkg_config_packages(mav-kmcl
    mav-state-est
    mav-est-fovis
    gpf-laser-lib
    lcmtypes_kinect
#    mav-est-legodo
#    mav-est-legodo-external
#    lcmtypes_drc_lcmtypes
#    model-client  
    gpf-rgbd-lib)

# don't know why this is necessary (something to do with concise args):
target_link_libraries(mav-kmcl
    boost_system)
pods_install_executables(mav-kmcl)
