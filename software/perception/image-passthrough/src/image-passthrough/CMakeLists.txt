

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(GLEW)
find_package(GLUT)
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})
set(REQUIRED_LIBS ${VTK_IO_TARGET_LINK_LIBRARIES} ${GLEW_LIBRARIES}    
       ${GLUT_LIBRARIES} vtkCommon vtkIO vtkHybrid)
set(PKG_CONFIG_DEPS eigen3 rgbd_simulation pcl_visualization-1.6 pcl_io-1.6)
include_directories(${VTK_INCLUDE_DIRS} ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR})


################
add_library(image-passthrough  SHARED image-passthrough.cpp)
pods_use_pkg_config_packages(image-passthrough ${PCL_LIBRARIES}
  ${PKG_CONFIG_DEPS}  opencv
  lcmtypes_bot2-core
  eigen3)
target_link_libraries (image-passthrough boost_system 
           ${REQUIRED_LIBS})
set_target_properties(image-passthrough PROPERTIES SOVERSION 1)
pods_install_libraries(image-passthrough)
pods_install_headers( image-passthrough.hpp DESTINATION image-passthrough)
pods_install_pkg_config_file(image-passthrough
  LIBS -limage-passthrough
  VERSION 0.0.1)



################
add_library(image-passthrough-app  SHARED image-passthrough-app.cpp)
pods_use_pkg_config_packages(image-passthrough-app ${PCL_LIBRARIES}
  ${PKG_CONFIG_DEPS}  opencv collision_object_gfe visualization-utils
  lcmtypes_bot2-core pointcloud_vis image-passthrough bot2-param-client
  image_io_utils   rgbd_primitives  affordance-utils
  eigen3)
target_link_libraries (image-passthrough-app boost_system 
           ${REQUIRED_LIBS})
set_target_properties(image-passthrough-app PROPERTIES SOVERSION 1)
pods_install_libraries(image-passthrough-app)
pods_install_headers( image-passthrough-app.hpp DESTINATION image-passthrough)
pods_install_pkg_config_file(image-passthrough-app
  LIBS -limage-passthrough-app 
  VERSION 0.0.1)


add_executable(drc-image-passthrough image-passthrough-main.cpp)
target_link_libraries(drc-image-passthrough boost_system   
           ${REQUIRED_LIBS})
pods_use_pkg_config_packages(drc-image-passthrough  bot2-param-client
       image-passthrough-app lcm)
pods_install_executables(drc-image-passthrough)
