cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(local_map)
add_definitions(    -ggdb3     -std=gnu99    )


set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(GLEW)
find_package(GLUT)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
set(REQUIRED_LIBS ${VTK_IO_TARGET_LINK_LIBRARIES} ${GLEW_LIBRARIES}    
       ${GLUT_LIBRARIES} vtkCommon vtkIO vtkHybrid)
set(PKG_CONFIG_DEPS eigen3 rgbd_simulation pcl_visualization-1.6 pcl_io-1.6)
include_directories(${VTK_INCLUDE_DIRS} ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})



set(PCL_LIBRARIES  pcl_io-1.6)
set(REQUIRED_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics)

add_executable(drc-renderer-sandia-palm renderer_sandia_palm.cpp  )
pods_use_pkg_config_packages(drc-renderer-sandia-palm
    path-util model-client   bot2-param-client
        pointcloud_lcm        pointcloud_vis        pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
       ${PCL_LIBRARIES} ${REQUIRED_PACKAGES})
target_link_libraries (drc-renderer-sandia-palm boost_system 
           ${REQUIRED_LIBS})
pods_install_executables(drc-renderer-sandia-palm )
