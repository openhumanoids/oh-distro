add_definitions(-Wall -fPIC -m64 -enable-shared  -enable-pic)
add_library(pointcloud_math  SHARED pointcloud_math.cpp)

target_link_libraries(pointcloud_math jpeg ${LCMTYPES_LIBS})
pods_use_pkg_config_packages(pointcloud_math
    eigen3
    )
#  lcm eigen3   bot2-core) pcl_common glib-2.0)

set_target_properties(pointcloud_math PROPERTIES SOVERSION 1)
pods_install_libraries(pointcloud_math)
pods_install_headers(pointcloud_math.hpp DESTINATION pointcloud_tools)
pods_install_pkg_config_file(pointcloud_math
    LIBS -lpointcloud_math
    VERSION 0.0.1)

#    REQUIRES bot2-vis lcm visualization lcmtypes_kinect lcmtypes_openni_utils pcl_io cminpack

# removed: lcmtypes_openni-lcm
