cmake_minimum_required(VERSION 2.6.0)

project(tld-tracker)
set(PACKAGE_NAME tld-tracker)

find_package(PkgConfig REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -fPIC -std=c++0x -Wno-deprecated-declarations")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgomp")


include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
)

add_subdirectory(tld)
add_subdirectory(cvblobs)

include_directories(tld tld/tracker)
link_libraries(tld)

# ======================================================
# DRC tld-tracker library
# ======================================================

# Create a shared library
add_library(
  ${PACKAGE_NAME} SHARED
  tld-tracker.cpp
  )

# make the header public and install it
pods_install_headers(
  tld-tracker.hpp
  DESTINATION ${PACKAGE_NAME})

# make the library public
pods_install_libraries(${PACKAGE_NAME})

set(REQUIRED_PACKAGES lcm glib-2.0 bot2-core image-utils bot2-lcmgl-client 
  bot2-frames bot2-param-client opencv image_io_utils perception-opencv-utils)
pods_use_pkg_config_packages ( ${PACKAGE_NAME} ${REQUIRED_PACKAGES})
# create a pkg-config file for the library, to make it easier for other
# software to use.
pods_install_pkg_config_file(${PACKAGE_NAME}
    CFLAGS
    LIBS -l${PACKAGE_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)

# ======================================================
# DRC tld-tracker test app
# ======================================================

# create an executable, and make it public
add_executable(drc-tld-tracker 
  tld-tracker-app.cpp
  tld-tracker.cpp
)

pods_use_pkg_config_packages ( drc-tld-tracker lcm glib-2.0 lcmtypes_kinect bot2-core image-utils bot2-lcmgl-client 
  bot2-frames
  bot2-param-client opencv image_io_utils perception-opencv-utils)


target_link_libraries(drc-tld-tracker 
  ${LCMTYPES_LIBS} 
  ${EIGEN_LIBRARIES} 
  ${FLANN_LIBRARIES} 
)

pods_install_executables(drc-tld-tracker)

# ======================================================
# DRC tld-segmenter app
# ======================================================

# create an executable, and make it public
add_executable(drc-tld-segmenter 
  tld-segmenter-app.cpp
)

pods_use_pkg_config_packages ( drc-tld-segmenter lcm glib-2.0 lcmtypes_kinect bot2-core image-utils bot2-lcmgl-client 
  bot2-frames
  bot2-param-client opencv perception-opencv-utils)


target_link_libraries(drc-tld-segmenter 
  ${LCMTYPES_LIBS} 
  ${LCMTYPES_LIBS} 
  ${EIGEN_LIBRARIES} 
  ${FLANN_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  ${GLU_LIBRARIES}
  ${GLUT_LIBRARIES}
)

pods_install_executables(drc-tld-segmenter)



# ======================================================
# DRC tld-driving-tracker app
# ======================================================

# create an executable, and make it public
add_executable(drc-driving-tracker 
  driving-tracker-app.cpp
  tld-tracker.cpp
)

pods_use_pkg_config_packages ( drc-driving-tracker lcm glib-2.0 lcmtypes_kinect bot2-core image-utils bot2-lcmgl-client 
  bot2-frames bot2-frames_cpp
  bot2-param-client opencv perception-opencv-utils)


target_link_libraries(drc-driving-tracker 
  ${LCMTYPES_LIBS} 
  ${EIGEN_LIBRARIES} 
  ${FLANN_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  ${GLU_LIBRARIES}
  ${GLUT_LIBRARIES}
)

pods_install_executables(drc-driving-tracker)
