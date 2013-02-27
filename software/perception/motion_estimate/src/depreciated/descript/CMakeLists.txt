cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
#project(local_map)



cmake_minimum_required ( VERSION 2.8 )
project( drc-descript )

#set( CMAKE_INSTALL_PREFIX /usr/local )
#set( CMAKE_CXX_COMPILER "g++-4.4" )
#set( CMAKE_CXX_FLAGS "-L /usr/local/lib" )

#include_directories( .
#                     /usr/local/include )

set ( OpenCV_LIBS opencv_core
                  opencv_highgui
                  opencv_nonfree
		opencv_calib3d
		opencv_imgproc
	                  opencv_features2d )




add_executable(drc-descript descript.cpp  )
target_link_libraries(drc-descript ${OpenCV_LIBS} )
pods_install_executables(drc-descript )






add_executable(drc-descript-brief descript_brief.cpp  )
target_link_libraries(drc-descript-brief ${OpenCV_LIBS} boost_thread)
pods_use_pkg_config_packages(drc-descript-brief eigen3
        pointcloud_lcm        pointcloud_vis        pointcloud_math
        lcmtypes_visualization
			bot2-lcmgl-client estimate-pose)
pods_install_executables(drc-descript-brief )

