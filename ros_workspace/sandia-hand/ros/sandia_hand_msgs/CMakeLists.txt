cmake_minimum_required(VERSION 2.8.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
if (NOT SANDIA_HAND_VERSION)
  message("WOAH THERE PARTNER. no version supplied. don't install this.")
  set(SANDIA_HAND_VERSION bogus) # only used for non-install builds
endif()
set(PKG_NAME sandia_hand_msgs)
set(ENV{ROS_PACKAGE_PATH} ${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
rosbuild_init()
#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
rosbuild_genmsg()
rosbuild_gensrv()

set(INSTALL_SHARE_PATH ${CMAKE_INSTALL_PREFIX}/share/sandia-hand-${SANDIA_HAND_VERSION}/ros/${PKG_NAME})

install(FILES manifest.xml
        DESTINATION ${INSTALL_SHARE_PATH})
install(DIRECTORY srv srv_gen msg msg_gen src DESTINATION ${INSTALL_SHARE_PATH} PATTERN ".*" EXCLUDE)
