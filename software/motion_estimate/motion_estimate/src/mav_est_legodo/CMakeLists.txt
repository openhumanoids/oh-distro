# added fro joint_utils vector assignment:
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

file(GLOB hpp_files *.hpp)

set(REQUIRED_LIBS mav-state-est 
    bot2-core 	bot2-param-client
    bot2-frames	eigen-utils
    path-util   lcmtypes_drc_lcmtypes)

# make the headers public
pods_install_headers(${hpp_files} DESTINATION mav_state_est/mav-est-legodo)

# make library that computes leg odo internally
add_library(mav-est-legodo SHARED 
                rbis_legodo_update.cpp
                rbis_legodo_common.cpp)

pods_use_pkg_config_packages(mav-est-legodo ${REQUIRED_LIBS} 
    leg_estimate torque_adjustment_tools
    bot2-frames_cpp drc-utils)
pods_install_libraries(mav-est-legodo)
pods_install_pkg_config_file(mav-est-legodo
    CFLAGS
    LIBS -lmav-est-legodo -lpcl_io
    REQUIRES 	${REQUIRED_LIBS} pcl_io-1.7
    VERSION 0.0.1)





# make library which receives external delta message
add_library(mav-est-legodo-external SHARED 
                rbis_legodo_external_update.cpp
                rbis_legodo_common.cpp)

pods_use_pkg_config_packages(mav-est-legodo-external ${REQUIRED_LIBS} 
    leg_estimate)
pods_install_libraries(mav-est-legodo-external)
pods_install_pkg_config_file(mav-est-legodo-external
    CFLAGS
    LIBS -lmav-est-legodo-external
    REQUIRES 	${REQUIRED_LIBS}
    VERSION 0.0.1)
