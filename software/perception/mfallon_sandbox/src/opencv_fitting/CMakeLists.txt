add_definitions(
    -ggdb3 
    -std=gnu99
    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(opencv_fitting)

add_executable(contours_demo generalContours_demo2.cpp  )
pods_use_pkg_config_packages(contours_demo
    bot2-lcmgl-client opencv)
pods_install_executables(contours_demo )


add_executable(ellipse_demo demo1.cpp  )
pods_use_pkg_config_packages(ellipse_demo
    bot2-lcmgl-client opencv)
pods_install_executables(ellipse_demo )


add_executable(matcher_demo SURF_FlannMatcher.cpp  )
pods_use_pkg_config_packages(matcher_demo
    bot2-lcmgl-client opencv)
pods_install_executables(matcher_demo )



add_executable(features2d_demo features2d_demo.cpp  )
pods_use_pkg_config_packages(features2d_demo
    bot2-lcmgl-client opencv)
pods_install_executables(features2d_demo )




add_executable(video_homography_demo video_homography_demo.cpp )
pods_use_pkg_config_packages(video_homography_demo
    bot2-lcmgl-client opencv)
pods_install_executables(video_homography_demo )


