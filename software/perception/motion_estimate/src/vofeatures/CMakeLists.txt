add_definitions(-Wall )

add_library(vofeatures SHARED vofeatures.cpp )
pods_use_pkg_config_packages(vofeatures eigen3 
      libfovis   pointcloud_vis
      opencv)

set_target_properties(vofeatures PROPERTIES SOVERSION 1)
pods_install_libraries(vofeatures)
pods_install_headers(vofeatures.hpp  imagefeatures.hpp   DESTINATION drcvision)
pods_install_pkg_config_file(vofeatures
    LIBS -lvofeatures
    REQUIRES 
    VERSION 0.0.1)
