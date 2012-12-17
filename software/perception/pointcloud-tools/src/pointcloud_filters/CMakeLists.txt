################################################################
add_library(pointcloud_filters SHARED filter_planes.cpp grow_cloud.cpp 
	filter_findfloor.cpp)
#    filter_light.cpp filter_person.cpp 
#    filter_cloudtolaser.cpp)

pods_use_pkg_config_packages(pointcloud_filters   eigen3  
	pcl_common-1.6   pcl_io-1.6      pcl_surface-1.6   pcl_segmentation-1.6 
	pcl_filters-1.6  pcl_surface-1.6 pcl_io-1.6 	   pcl_kdtree-1.6  pcl_search-1.6)

target_link_libraries(pointcloud_filters)
# kmcl_utils qhull ${LCMTYPES_LIBS} pcl_common pcl_octree)
pods_use_pkg_config_packages(pointcloud_filters lcm eigen3   glib-2.0 )
# kmcl_utils 

# set the library API version.  Increment this every time the public API changes.
set_target_properties(pointcloud_filters PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_filters)
pods_install_headers(filter_planes.hpp grow_cloud.hpp  
    filter_findfloor.hpp filter_colorize.hpp
    DESTINATION pointcloud_tools)
# filter_light.hpp 
#    filter_person.hpp   filter_cloudtolaser.hpp

pods_install_pkg_config_file(pointcloud_filters
    LIBS -lpointcloud_filters
    REQUIRES bot2-vis lcm  lcmtypes_visualization pointcloud_math
    VERSION 0.0.1) 
# visualization 


