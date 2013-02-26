cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME stereo-bm)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

pkg_check_modules(OPENGL REQUIRED gl)

include_directories(
  ${GLUT_INCLUDE_DIRS}
  ${GLU_INCLUDE_DIRS}
)

# create an executable, and make it public
add_executable(stereo-bm 
  src/stereo_match.cc
)

pods_use_pkg_config_packages ( stereo-bm lcm glib-2.0 lcmtypes_kinect bot2-core image-utils bot2-lcmgl-client 
  bot2-frames jpeg-utils
  bot2-param-client opencv perception-opencv-utils lcmtypes_multisense )

target_link_libraries(stereo-bm 
  ${LCMTYPES_LIBS} 
  ${OPENGL_LIBRARIES}
  ${GLU_LIBRARIES}
  ${GLUT_LIBRARIES}
)
pods_install_executables(stereo-bm)
