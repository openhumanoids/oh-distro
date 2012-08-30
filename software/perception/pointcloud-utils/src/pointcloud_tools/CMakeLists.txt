################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pointcloud_tools)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(PCL 1.5 REQUIRED COMPONENTS io visualization common filters simulation  segmentation)
#find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS} ${GLUT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR}  /usr/include/eigen3)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(pointcloud_tools SHARED pointcloud_tools.cpp)


set(EXAMPLE_LIBRARIES  ${PCL_IO_LIBRARIES} 
                       ${PCL_VISUALIZATION_LIBRARIES} 
                       ${PCL_COMMON_LIBRARIES} 
                       ${PCL_FILTERS_LIBRARIES}
                       ${GLEW_LIBRARIES}
                       ${GLUT_LIBRARIES}
                       ${OPENGL_LIBRARIES})



target_link_libraries(pointcloud_tools ${LCMTYPES_LIBS}
		${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(pointcloud_tools  lcm eigen3   bot2-core  glib-2.0  )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pointcloud_tools PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_tools)
pods_install_headers(pointcloud_tools.hpp DESTINATION pointcloud_utils)
pods_install_pkg_config_file(pointcloud_tools
    LIBS -lpointcloud_tools
    REQUIRES bot2-vis
    VERSION 0.0.1)

