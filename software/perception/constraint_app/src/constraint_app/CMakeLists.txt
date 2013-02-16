add_definitions(
    -ggdb3 
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(constraint_app)

#set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
#	pcl_segmentation-1.6 pcl_sample_consensus-1.6
#	pcl_io-1.6)

set(KDL_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector collision_object eigen3) 

add_executable(constraint_app constraint_app.cpp )
#target_link_libraries (drc-map-store-velodyne ${EXAMPLE_LIBRARIES})
pods_use_pkg_config_packages(constraint_app
    lcmtypes_velodyne
    path-util
    bot2-param-client
    lcmtypes_drc_lcmtypes  lcmtypes_visualization otdf_dom
    ${KDL_PACKAGES})

pods_install_executables(constraint_app )
