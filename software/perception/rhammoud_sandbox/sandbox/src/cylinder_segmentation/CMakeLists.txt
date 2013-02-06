add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(PCL_LIBRARIES pcl_common-1.6 pcl_io-1.6 pcl_visualization-1.6)

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(cylinder_segmentation cylinder_segmentation.cpp  )
pods_use_pkg_config_packages(cylinder_segmentation  ${PCL_LIBRARIES})

target_link_libraries(cylinder_segmentation boost_thread)
include_directories(cylinder_segmentation vtk-5.6)
pods_install_executables(cylinder_segmentation )
