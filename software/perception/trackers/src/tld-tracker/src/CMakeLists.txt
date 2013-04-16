cmake_minimum_required(VERSION 2.6.0)

find_package(PkgConfig REQUIRED)

pkg_check_modules(EIGEN REQUIRED eigen3)
# pkg_check_modules(OPENGL REQUIRED gl)
# pkg_check_modules(GLU REQUIRED glu)
# pkg_check_modules(GSL REQUIRED gsl)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp -fPIC")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp -fPIC")
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lgomp")


include_directories(
  # ${GSL_INCLUDE_DIRS}
  # ${GLUT_INCLUDE_DIRS}
  # ${GLU_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${FLANN_INCLUDE_DIRS}
)

add_subdirectory(tld)
add_subdirectory(cvblobs)

include_directories(tld tld/tracker)
link_libraries(tld)

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
  # ${OPENGL_LIBRARIES}
  # ${GLU_LIBRARIES}
  # ${GLUT_LIBRARIES}
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
  bot2-param-client opencv perception-opencv-utils )


target_link_libraries(drc-tld-segmenter 
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
  bot2-frames
  bot2-param-client opencv perception-opencv-utils )


target_link_libraries(drc-driving-tracker 
  ${LCMTYPES_LIBS} 
  ${EIGEN_LIBRARIES} 
  ${FLANN_LIBRARIES} 
  ${OPENGL_LIBRARIES}
  ${GLU_LIBRARIES}
  ${GLUT_LIBRARIES}
)

pods_install_executables(drc-driving-tracker)

# # create an executable, and make it public
# find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
# include_directories(${Boost_INCLUDE_DIRS})
# add_executable(multi-object-tracker 
#   multi_object_tracker.cpp
#   #linemod.cpp
#   #MRPTTracker.cc
#   #HistTracker.cc
# )


# pods_use_pkg_config_packages ( multi-object-tracker lcm glib-2.0 lcmtypes_kinect lcmtypes_multisense bot2-core image-utils bot2-lcmgl-client 
#   bot2-frames
#   bot2-param-client opencv perception-opencv-utils mrpt-base mrpt-vision mrpt-obs mrpt-opengl
#   pcl_common-1.6   pcl_io-1.6      pcl_surface-1.6   pcl_segmentation-1.6 
#   pcl_filters-1.6  pcl_surface-1.6 pcl_io-1.6 	   pcl_kdtree-1.6  pcl_search-1.6 image_io_utils  lcmtypes_visualization)

# target_link_libraries(multi-object-tracker 
#   ${LCMTYPES_LIBS} 
#   ${EIGEN_LIBRARIES} 
#   ${FLANN_LIBRARIES} 
#   ${OPENGL_LIBRARIES}
#   ${GLU_LIBRARIES}
#   ${GLUT_LIBRARIES}
#   ${Boost_LIBRARIES}
# )
# pods_install_executables(multi-object-tracker )
