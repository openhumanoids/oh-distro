cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_examples)

#set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(PCL 1.6 REQUIRED COMPONENTS io)# visualization common filters segmentation surface features)
#find_package(OpenGL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} /usr/include/eigen3)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(EXAMPLE_LIBRARIES  ${PCL_IO_LIBRARIES} 
                        pcl_segmentation
                        pcl_surface
                        pcl_features
                       ${PCL_VISUALIZATION_LIBRARIES} 
                       ${PCL_COMMON_LIBRARIES} 
                       ${PCL_FILTERS_LIBRARIES}
                       ${OPENGL_LIBRARIES})

add_executable(drc-local-map local_map.cpp   )
target_link_libraries (drc-local-map ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-local-map  pointcloud_vis
		lcmtypes_drc_lcmtypes  lcmtypes_visualization)
pods_install_executables(drc-local-map )
