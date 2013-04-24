set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS})

set(REQUIRED_LIBS vtkCommon vtkFiltering)
#${VTK_IO_TARGET_LINK_LIBRARIES} )
# vtkIO vtkHybrid)


set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
  pcl_segmentation-1.6 pcl_sample_consensus-1.6 pcl_visualization-1.6
  pcl_registration-1.6
  pcl_io-1.6)

add_executable(drc-icp-tutorial icp-tutorial.cpp)
pods_use_pkg_config_packages(drc-icp-tutorial ${PCL_LIBRARIES})
target_link_libraries (drc-icp-tutorial ${REQUIRED_LIBS})
pods_install_executables(drc-icp-tutorial)

#add_executable(drc-ndt-tutorial ndt-tutorial.cpp)
#pods_use_pkg_config_packages(drc-ndt-tutorial ${PCL_LIBRARIES})
#target_link_libraries (drc-ndt-tutorial ${REQUIRED_LIBS})
#pods_install_executables(drc-ndt-tutorial)



################ LIBRARIES ###############################
add_library(icp-tracker  SHARED icp-tracker.cpp)
pods_use_pkg_config_packages(icp-tracker ${PCL_LIBRARIES} bot2-lcmgl-client maps eigen3
     pointcloud_filters)
target_link_libraries (icp-tracker boost_system 
           ${REQUIRED_LIBS})
set_target_properties(icp-tracker PROPERTIES SOVERSION 1)
pods_install_libraries(icp-tracker)
pods_install_headers( icp-tracker.hpp DESTINATION trackers)
pods_install_pkg_config_file(icp-tracker
  LIBS -licp-tracker
  VERSION 0.0.1)



################ APPS ###############################

add_executable(drc-icp-tracker-standalone-app icp-tracker-standalone-app.cpp)
pods_use_pkg_config_packages(drc-icp-tracker-standalone-app 
  icp-tracker
  bot2-param-client bot2-lcmgl-client
  bot2-frames  bot2-frames_cpp 
  pointcloud_filters  pointcloud_vis pointcloud_lcm ${PCL_LIBRARIES})
pods_install_executables(drc-icp-tracker-standalone-app)




add_executable (drc-icp-tracker-app icp-tracker-app.cpp )
pods_use_pkg_config_packages(drc-icp-tracker-app
   bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  bot2-param-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_math 
  icp-tracker major-plane-detect affordance-utils
  ${PCL_LIBRARIES})
pods_install_executables(drc-icp-tracker-app)
