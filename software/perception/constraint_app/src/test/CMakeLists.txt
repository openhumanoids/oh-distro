add_definitions(-ggdb3)

find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ca_test)

#set(PCL_LIBRARIES  pcl_search-1.6 pcl_filters-1.6 pcl_features-1.6
#	pcl_segmentation-1.6 pcl_sample_consensus-1.6
#	pcl_io-1.6)

set(KDL_PACKAGES urdf orocos-kdl kdl-parser forward-kinematics otdf_dom collision_detector collision_object eigen3) 

add_executable(ca_test ca_test.cpp mytreefksolverpos_recursive.cpp)
#target_link_libraries (drc-map-store-velodyne ${EXAMPLE_LIBRARIES})
target_link_libraries(ca_test ${Boost_LIBRARIES})
pods_use_pkg_config_packages(ca_test
    lcmtypes_velodyne
    path-util
    bot2-param-client
    bot2-lcmgl-client
    lcmtypes_drc_lcmtypes  lcmtypes_visualization otdf_dom
    ${KDL_PACKAGES})

pods_install_executables(ca_test )

