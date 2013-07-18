cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(gen-mesh)

set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
	pcl_segmentation-1.6 pcl_sample_consensus-1.6
	pcl_io-1.6)





################################################################
add_library(gen-mesh SHARED gen-mesh.cpp)

pods_use_pkg_config_packages(gen-mesh eigen3 lcm glib-2.0 ${PCL_LIBRARIES})

# set the library API version.  Increment this every time the public API changes.
set_target_properties(gen-mesh PROPERTIES SOVERSION 1)
pods_install_libraries(gen-mesh)
pods_install_headers(gen-mesh.hpp
    DESTINATION gen-mesh)

pods_install_pkg_config_file(gen-mesh
    LIBS -lgen-mesh
    REQUIRES bot2-vis lcm  lcmtypes_visualization
    VERSION 0.0.1) 





add_executable(drc-gen-mesh-app gen-mesh-app.cpp   )
pods_use_pkg_config_packages(drc-gen-mesh-app
        pointcloud_lcm         pointcloud_vis         pointcloud_math
	lcmtypes_drc_lcmtypes  lcmtypes_visualization
        ${PCL_LIBRARIES} gen-mesh)
pods_install_executables(drc-gen-mesh-app)
