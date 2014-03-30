set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# install it to include/drc_utils
pods_install_headers(joint_utils.hpp DESTINATION drc_utils)

pods_install_headers(frame_check_utils.hpp DESTINATION drc_utils)
