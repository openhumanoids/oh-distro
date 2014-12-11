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

add_executable(drc-perception-example perception_example.cpp    )
target_link_libraries (drc-perception-example ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-perception-example  
		lcmtypes_drc_lcmtypes  lcmtypes_visualization)
pods_install_executables(drc-perception-example )


add_executable(drc-perception-example-save perception_example_save.cpp    )
target_link_libraries (drc-perception-example-save ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(drc-perception-example-save  
		lcmtypes_drc_lcmtypes  )
pods_install_executables(drc-perception-example-save )
