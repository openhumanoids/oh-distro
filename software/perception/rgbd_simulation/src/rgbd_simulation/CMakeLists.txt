    set(SRCS 
         camera.cpp
         model.cpp
         range_likelihood.cpp
         scene.cpp
         glsl_shader.cpp
         sum_reduce.cpp
        )

    set(INCS
        camera.h
        model.h
        range_likelihood.h
        scene.h
        glsl_shader.h
        sum_reduce.h
        )

set(PCL_LIBRARIES  pcl_io-1.6)


add_library(rgbd_simulation  SHARED ${SRCS})

target_link_libraries(rgbd_simulation)
pods_use_pkg_config_packages(rgbd_simulation ${PCL_LIBRARIES}
    eigen3
    )
#  lcm eigen3   bot2-core) pcl_common glib-2.0)




set_target_properties(rgbd_simulation PROPERTIES SOVERSION 1)
pods_install_libraries(rgbd_simulation)
pods_install_headers( ${INCS} DESTINATION rgbd_simulation)
pods_install_pkg_config_file(rgbd_simulation
    LIBS -lrgbd_simulation
    VERSION 0.0.1)

#    REQUIRES bot2-vis lcm visualization lcmtypes_kinect lcmtypes_openni_utils pcl_io cminpack

# removed: lcmtypes_openni-lcm
