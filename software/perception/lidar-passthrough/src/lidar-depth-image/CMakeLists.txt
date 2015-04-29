set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

add_executable(drc-get-lidar-depth-image get-lidar-depth-image.cpp)
target_link_libraries(drc-get-lidar-depth-image boost_system)
pods_use_pkg_config_packages(drc-get-lidar-depth-image maps bot2-lcmgl-client
              pronto_vis bot2-frames bot2-frames_cpp
              lcmtypes_multisense
              camera_params
              image_io_utils)
pods_install_executables(drc-get-lidar-depth-image)



add_executable(drc-get-lidar-depth-image-minimal get-lidar-depth-image-minimal.cpp)
target_link_libraries(drc-get-lidar-depth-image-minimal boost_system)
pods_use_pkg_config_packages(drc-get-lidar-depth-image-minimal maps bot2-lcmgl-client
              pronto_vis bot2-frames bot2-frames_cpp
              lcmtypes_multisense
              camera_params
              image_io_utils)
pods_install_executables(drc-get-lidar-depth-image-minimal)


add_executable(drc-colorize-lidar-scan colorize-lidar-scan.cpp)
target_link_libraries(drc-colorize-lidar-scan boost_system)
pods_use_pkg_config_packages(drc-colorize-lidar-scan maps bot2-lcmgl-client
              pronto_vis bot2-frames bot2-frames_cpp
              lcmtypes_multisense
              camera_params
              image_io_utils)
pods_install_executables(drc-colorize-lidar-scan)

