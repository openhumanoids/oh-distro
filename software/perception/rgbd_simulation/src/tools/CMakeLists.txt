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
add_executable(rgbd-sim-viewer sim_viewer.cpp)
target_link_libraries(rgbd-sim-viewer boost_system ${REQUIRED_LIBS})
pods_use_pkg_config_packages(rgbd-sim-viewer ${PKG_CONFIG_DEPS})
pods_install_executables(rgbd-sim-viewer)

################
add_executable(rgbd-sim-test-simple sim_test_simple.cpp)
target_link_libraries(rgbd-sim-test-simple boost_system boost_thread  
           ${REQUIRED_LIBS})
pods_use_pkg_config_packages(rgbd-sim-test-simple ${PKG_CONFIG_DEPS})
pods_install_executables(rgbd-sim-test-simple)

################
add_executable(rgbd-sim-test-performance sim_test_performance.cpp)
target_link_libraries (rgbd-sim-test-performance boost_system
           ${REQUIRED_LIBS})
pods_use_pkg_config_packages(rgbd-sim-test-performance ${PKG_CONFIG_DEPS})
pods_install_executables(rgbd-sim-test-performance)

################
add_library(rgbd_simulation_io  SHARED simulation_io.cpp)
pods_use_pkg_config_packages(rgbd_simulation_io ${PCL_LIBRARIES}
  ${PKG_CONFIG_DEPS}  opencv
  eigen3)
target_link_libraries (rgbd_simulation_io boost_system 
           ${REQUIRED_LIBS})
set_target_properties(rgbd_simulation_io PROPERTIES SOVERSION 1)
pods_install_libraries(rgbd_simulation_io)
pods_install_headers( simulation_io.hpp DESTINATION rgbd_simulation_io)
pods_install_pkg_config_file(rgbd_simulation_io
  LIBS -lrgbd_simulation_io
  VERSION 0.0.1)

################
add_executable(rgbd-sim-terminal-demo ${SUBSYS_NAME} sim_terminal_demo.cpp)
target_link_libraries (rgbd-sim-terminal-demo boost_system 
           ${REQUIRED_LIBS})
pods_use_pkg_config_packages(rgbd-sim-terminal-demo ${PKG_CONFIG_DEPS} 
           rgbd_simulation_io )
pods_install_executables(rgbd-sim-terminal-demo)
