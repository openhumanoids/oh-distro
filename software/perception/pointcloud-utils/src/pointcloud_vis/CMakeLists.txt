################################################################
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(pcl_examples)

option(BUILD_CUDA_examples "Build examples that use cuda" OFF)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake/Modules/" $CMAKE_MODULE_PATH})
find_package(PCL 1.5 REQUIRED COMPONENTS io visualization common filters simulation  segmentation)
find_package(GLEW REQUIRED)
find_package(GLUT REQUIRED)
find_package(OpenGL REQUIRED)

if (BUILD_CUDA_examples)
  find_package(CUDA REQUIRED)
endif (BUILD_CUDA_examples)

include_directories(${PCL_INCLUDE_DIRS} ${GLEW_INCLUDE_DIR} ${GLUT_INCLUDE_DIR} ${OPENGL_INCLUDE_DIR} ${CUDA_SDK_ROOT_DIR}/CUDALibraries/common/inc ${CUDA_SDK_ROOT_DIR}/shared/inc /usr/include/eigen3)
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_library(kmcl_utils SHARED kmcl_utils.cpp)


set(EXAMPLE_LIBRARIES  ${PCL_IO_LIBRARIES} 
                       ${PCL_VISUALIZATION_LIBRARIES} 
                       ${PCL_COMMON_LIBRARIES} 
                       ${PCL_FILTERS_LIBRARIES}
                       ${GLEW_LIBRARIES}
                       ${GLUT_LIBRARIES}
                       ${OPENGL_LIBRARIES})



target_link_libraries(kmcl_utils jpeg ${LCMTYPES_LIBS}
		${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(kmcl_utils  lcm eigen3   bot2-core  glib-2.0  )

# set the library API version.  Increment this every time the public API changes.
set_target_properties(kmcl_utils PROPERTIES SOVERSION 1)
pods_install_libraries(kmcl_utils)
pods_install_headers(kmcl_utils.hpp DESTINATION kmcl)
pods_install_pkg_config_file(kmcl_utils
    LIBS -lkmcl_utils
    REQUIRES bot2-vis kinect-utils jpeg_utils 
    VERSION 0.0.1)

