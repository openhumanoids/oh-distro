add_executable(drc-image-splitter image-splitter.cpp  )
pods_use_pkg_config_packages(drc-image-splitter
                lcmtypes_bot2-core
                opencv)
pods_install_executables(drc-image-splitter )




set(PCL_LIBRARIES  pcl_io-1.6)


add_executable(drc-image-writer image-writer.cpp  )
pods_use_pkg_config_packages(drc-image-writer
                lcmtypes_bot2-core
                ${PCL_LIBRARIES}
                pointcloud_vis
                opencv)
pods_install_executables(drc-image-writer )


add_executable(drc-multisense-image-tool multisense-image-tool.cpp  )
pods_use_pkg_config_packages(drc-multisense-image-tool
                lcmtypes_bot2-core
                lcmtypes_multisense
                ${PCL_LIBRARIES}
                pointcloud_vis
                pointcloud_lcm
                bot2-core
                bot2-frames
                bot2-frames_cpp
                opencv)
pods_install_executables(drc-multisense-image-tool )



add_executable(drc-vicon2pose vicon2pose.cpp  )
pods_use_pkg_config_packages(drc-vicon2pose
                lcmtypes_bot2-core
                lcmtypes_multisense
                ${PCL_LIBRARIES}
                pointcloud_vis
                opencv)
pods_install_executables(drc-vicon2pose )
