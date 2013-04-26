set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
set(PCL_LIBRARIES  pcl_segmentation-1.6 pcl_sample_consensus-1.6  pcl_io-1.6)


################ LIBRARIES ###############################
add_library(stereo-bm  SHARED stereo-bm.cpp)
pods_use_pkg_config_packages(stereo-bm 
   image_io_utils bot2-param-client lcmtypes_multisense bot2-core
   eigen3 opencv)
target_link_libraries (stereo-bm boost_system 
           ${REQUIRED_LIBS})
set_target_properties(stereo-bm PROPERTIES SOVERSION 1)
pods_install_libraries(stereo-bm)
pods_install_headers( stereo-bm.hpp DESTINATION stereo-bm)
pods_install_pkg_config_file(stereo-bm
  LIBS -lstereo-bm
  VERSION 0.0.1)

  
############# INDIVIDUAL APPS #########################  
add_executable (drc-stereo-bm-app stereo-bm-app.cpp )
target_link_libraries(drc-stereo-bm-app  particle)
pods_use_pkg_config_packages(drc-stereo-bm-app  stereo-bm
  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
  pointcloud_vis pointcloud_math opencv image_io_utils)
pods_install_executables(drc-stereo-bm-app )

add_executable (drc-stereo-bm-cloud stereo-bm-cloud.cpp )
target_link_libraries(drc-stereo-bm-cloud  particle)
pods_use_pkg_config_packages(drc-stereo-bm-cloud  stereo-bm
  bot2-core lcmtypes_visualization  lcmtypes_bot2-core
                pointcloud_lcm
  pointcloud_vis pointcloud_math opencv image_io_utils)
pods_install_executables(drc-stereo-bm-cloud )
