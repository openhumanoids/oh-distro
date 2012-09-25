cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
add_definitions(-Wall -std=gnu99)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer surrogate")
#    return()
#endif(NOT BOT2_VIS_FOUND)

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

set(PCL_LIBRARIES pcl_io-1.3 pcl_filters-1.3
    pcl_segmentation-1.3 pcl_sample_consensus-1.3)

include_directories(. perception)
include_directories(. display_objects)

add_library(surrogate SHARED 
  SurrogateRenderer.cc
  UIProcessing.cc
  LinearAlgebra.cc
  display_objects/Color.cc
  display_objects/ObjectPoints.cc
  display_objects/AdjustableVector.cc
  perception/ObjectTracker.cc
  perception/Segmentation.cc
  perception/PclSurrogateUtils.cc
  perception/EdgesToR3.cc
  perception/contour_classification/ContourUtils.cc
  )

target_link_libraries(surrogate
    ${OPENGL_LIBRARIES}
    ${OpenCV_LIBS}
    )

pods_use_pkg_config_packages(surrogate
    ${PCL_LIBRARIES}
    bot2-vis
    bot2-frames
    bot2-param-client)

pods_install_headers(SurrogateRenderer.h UIProcessing.h SurrogateException.h LinearAlgebra.h 
					display_objects/DisplayInfo.h display_objects/Color.h 
					display_objects/ObjectPoints.h display_objects/AdjustableVector.h
					perception/ObjectTracker.h perception/Segmentation.h perception/PclSurrogateUtils.h
    				perception/EdgesToR3.h
    				perception/contour_classification/ContourUtils.h
    				DESTINATION surrogate)

pods_install_libraries(surrogate)

pods_install_pkg_config_file(surrogate
    LIBS -lsurrogate -l${LCMTYPES_LIBS}
    REQUIRES bot2-vis bot2-frames bot2-param-client)
pods_install_libraries(surrogate)

#=============#=============#=============#=============#=============#=============#=============
add_executable(surrogate_viewer main.cc)
pods_use_pkg_config_packages(surrogate_viewer
    glib-2.0
    bot2-vis
    bot2-frames
    bot2-param-client
    lcm
    surrogate)

pods_install_executables(surrogate_viewer)
