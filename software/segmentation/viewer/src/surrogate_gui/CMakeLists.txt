cmake_minimum_required(VERSION 2.6 FATAL_ERROR)
add_definitions(-Wall -std=gnu99)
#if(NOT BOT2_VIS_FOUND)
#    message("bot2-vis not found.  Not building libbot2 renderer surrogate")
#    return()
#endif(NOT BOT2_VIS_FOUND)

# required for affordance-utils
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

find_package(PkgConfig REQUIRED)
find_package(OpenGL REQUIRED)

set(PCL_LIBRARIES pcl_io-1.6 pcl_filters-1.6
    pcl_segmentation-1.6 pcl_sample_consensus-1.6
    pcl_features-1.6 pcl_visualization-1.6)

include_directories(. perception)
include_directories(. display_objects)
include_directories(perception/pcl_circle3d/include)

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
	perception/pcl_circle3d/src/sac_model_circle3d.cpp
        perception/distTransform.cpp
        perception/PointCloudFitting.cpp
#  otdf_renderer/renderer_otdf.cpp
#  otdf_renderer/AffordanceCollectionListener.cpp
#  otdf_renderer/RobotStateListener.cpp
  )

target_link_libraries(surrogate
    ${OPENGL_LIBRARIES}
    )

set(REQUIRED_DRC_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom affordance)
pods_use_pkg_config_packages(surrogate
    ${PCL_LIBRARIES}
    bot2-vis
    bot2-frames
    bot2-param-client
    bot2-lcmgl-client
    lcmtypes_drc_lcmtypes
    maps
    pointcloud_filters pointcloud_vis pointcloud_math affordance-utils
    ${REQUIRED_DRC_PACKAGES})

pods_install_headers(SurrogateRenderer.h UIProcessing.h SurrogateException.h LinearAlgebra.h 
					display_objects/DisplayInfo.h display_objects/Color.h 
					display_objects/ObjectPoints.h display_objects/AdjustableVector.h
					perception/ObjectTracker.h perception/Segmentation.h perception/PclSurrogateUtils.h
                                        perception/PointCloudFitting.h
#					otdf_renderer/AffordanceCollectionListener.hpp
#					otdf_renderer/RobotStateListener.hpp
#					otdf_renderer/renderer_otdf.hpp
    					DESTINATION surrogate)

pods_install_libraries(surrogate)

pods_install_pkg_config_file(surrogate
    LIBS -lsurrogate -l${LCMTYPES_LIBS}
    REQUIRES bot2-vis bot2-frames bot2-param-client)
pods_install_libraries(surrogate)

#=============#=============#=============#=============#=============#=============#=============
add_executable(segmentation-viewer main.cc)
pods_use_pkg_config_packages(segmentation-viewer
    glib-2.0
    bot2-vis
    bot2-frames
    bot2-param-client
    bot2-lcmgl-client
    bot2-lcmgl-renderer
    lcm
    surrogate
    renderer_affordances
        renderer_drcgrid
    pointcloud_vis pointcloud_math)

pods_install_executables(segmentation-viewer)

#########################################################33

add_executable(testfitting perception/testfitting.cpp perception/PointCloudFitting.cpp )

#set_target_properties( testfitting PROPERTIES COMPILE_FLAGS "-O0 -DDEBUG -g ")

pods_use_pkg_config_packages(testfitting
    glib-2.0
    bot2-vis
    bot2-frames
    bot2-param-client
    bot2-lcmgl-client
    bot2-lcmgl-renderer
    lcm
    surrogate
    renderer_affordances
        renderer_drcgrid
    pointcloud_vis pointcloud_math)

pods_install_executables(testfitting)
