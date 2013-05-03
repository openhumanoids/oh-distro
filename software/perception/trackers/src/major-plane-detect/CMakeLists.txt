set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)

### needed to read obj:
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

################ LIBRARIES ###############################
add_library(major-plane-detect  SHARED major-plane-detect.cpp)
pods_use_pkg_config_packages(major-plane-detect ${PCL_LIBRARIES} bot2-lcmgl-client maps eigen3
     pointcloud_filters)
target_link_libraries (major-plane-detect boost_system 
           ${REQUIRED_LIBS})
set_target_properties(major-plane-detect PROPERTIES SOVERSION 1)
pods_install_libraries(major-plane-detect)
pods_install_headers( major-plane-detect.hpp DESTINATION trackers)
pods_install_pkg_config_file(major-plane-detect
  LIBS -lmajor-plane-detect
  VERSION 0.0.1)
  
  
############# INDIVIDUAL APPS #########################  
  
add_executable (drc-major-plane-tracker major-plane-tracker-app.cpp )
target_link_libraries(drc-major-plane-tracker boost_system)
pods_use_pkg_config_packages(drc-major-plane-tracker
   bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  bot2-param-client
  bot2-frames  bot2-frames_cpp
  pointcloud_vis pointcloud_math 
  major-plane-detect 
  affordance-utils
  ${PCL_LIBRARIES})
pods_install_executables(drc-major-plane-tracker)
