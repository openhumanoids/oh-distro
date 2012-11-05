cmake_minimum_required(VERSION 2.6.0)

find_package(PkgConfig REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)
pkg_check_modules(OPENGL REQUIRED gl)
pkg_check_modules(GLU REQUIRED glu)
pkg_check_modules(GSL REQUIRED gsl)


include_directories(
  ${GSL_INCLUDE_DIRS}
  ${GLUT_INCLUDE_DIRS}
  ${GLU_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
)

add_subdirectory(tld)
add_subdirectory(cvblobs)
include_directories(tld tld/tracker)
link_libraries(tld)

# create an executable, and make it public
add_executable(tld-tracker 
  tld_tracker.cc
)

pods_use_pkg_config_packages ( tld-tracker lcm glib-2.0 lcmtypes_kinect bot2-core image-utils bot2-lcmgl-client 
  bot2-frames
  bot2-param-client opencv perception-opencv-utils )


target_link_libraries(tld-tracker 
  ${LCMTYPES_LIBS} 
  ${EIGEN_LIBRARIES} 
  ${FLANN_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  ${GLU_LIBRARIES}
  ${GLUT_LIBRARIES}
)

pods_install_executables(tld-tracker)
