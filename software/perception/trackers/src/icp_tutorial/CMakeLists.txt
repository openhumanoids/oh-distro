set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

find_package(VTK REQUIRED)

include_directories(${VTK_INCLUDE_DIRS})

set(REQUIRED_LIBS vtkCommon vtkFiltering)
#${VTK_IO_TARGET_LINK_LIBRARIES} )
# vtkIO vtkHybrid)


set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
  pcl_segmentation-1.6 pcl_sample_consensus-1.6 pcl_visualization-1.6
  pcl_registration-1.6
  pcl_io-1.6)

add_executable(drc-icp-tutorial icp_tutorial.cpp)
pods_use_pkg_config_packages(drc-icp-tutorial ${PCL_LIBRARIES})
target_link_libraries (drc-icp-tutorial ${REQUIRED_LIBS})
pods_install_executables(drc-icp-tutorial)

#add_executable(drc-ndt-tutorial ndt_tutorial.cpp)
#pods_use_pkg_config_packages(drc-ndt-tutorial ${PCL_LIBRARIES})
#target_link_libraries (drc-ndt-tutorial ${REQUIRED_LIBS})
#pods_install_executables(drc-ndt-tutorial)
