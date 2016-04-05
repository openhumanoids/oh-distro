cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(image-comms)

add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)


# image split
add_executable(drc-image-split ImageSplit.cpp)
pods_use_pkg_config_packages(drc-image-split drc-utils opencv)
target_link_libraries(drc-image-split)
pods_install_executables(drc-image-split)

# image transmit
add_executable(drc-image-transmit ImageTransmit.cpp)
pods_use_pkg_config_packages(drc-image-transmit drc-utils opencv)
target_link_libraries(drc-image-transmit)
pods_install_executables(drc-image-transmit)

# image receive
add_executable(drc-image-receive ImageReceive.cpp)
pods_use_pkg_config_packages(drc-image-receive drc-utils opencv)
target_link_libraries(drc-image-receive)
pods_install_executables(drc-image-receive)

# video encoder
add_executable(drc-video-encode VideoEncode.cpp)
pods_use_pkg_config_packages(drc-video-encode drc-utils opencv x264 libswscale)
target_link_libraries(drc-video-encode)
pods_install_executables(drc-video-encode)

# video decoder
add_executable(drc-video-decode VideoDecode.cpp)
pods_use_pkg_config_packages(drc-video-decode drc-utils opencv x264 libswscale libavutil libavcodec)
target_link_libraries(drc-video-decode)
pods_install_executables(drc-video-decode)

# transmit webcam to lcm
add_executable(oh-webcam-driver WebcamToLCM.cpp)
pods_use_pkg_config_packages(oh-webcam-driver opencv lcm lcmtypes_bot2-core)
target_link_libraries(oh-webcam-driver)
pods_install_executables(oh-webcam-driver)