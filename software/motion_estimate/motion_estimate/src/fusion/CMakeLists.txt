set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(se-fusion fusion.cpp)
pods_use_pkg_config_packages(se-fusion
    mav-state-est
    mav-est-fovis
    gpf-laser-lib
#    lcmtypes_kinect
    mav-est-legodo
    mav-est-legodo-external
    lcmtypes_drc_lcmtypes
    model-client  
#    gpf-rgbd-lib
    mav-est-pose-init)

# don't know why this is necessary (something to do with concise args):
target_link_libraries(se-fusion
    boost_system)
pods_install_executables(se-fusion)
