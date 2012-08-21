################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pointcloud_lcm)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(PCL 1.5 REQUIRED COMPONENTS io visualization common filters simulation  segmentation)
#find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR}  /usr/include/eigen3)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(pointcloud_lcm SHARED pointcloud_lcm.cpp)


set(EXAMPLE_LIBRARIES  ${PCL_IO_LIBRARIES} 
                       ${PCL_VISUALIZATION_LIBRARIES} 
                       ${PCL_COMMON_LIBRARIES} 
                       ${PCL_FILTERS_LIBRARIES}
                       ${GLEW_LIBRARIES}
                       ${GLUT_LIBRARIES}
                       ${OPENGL_LIBRARIES})



target_link_libraries(pointcloud_lcm ${LCMTYPES_LIBS}
		${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(pointcloud_lcm  lcm eigen3   bot2-core  glib-2.0  )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pointcloud_lcm PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_lcm)
pods_install_headers(pointcloud_lcm.hpp DESTINATION pointcloud_utils)
pods_install_pkg_config_file(pointcloud_lcm
    LIBS -lpointcloud_lcm
    REQUIRES bot2-vis
    VERSION 0.0.1)

