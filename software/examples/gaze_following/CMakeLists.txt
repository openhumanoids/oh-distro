cmake_minimum_required(VERSION 2.6.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(POD_NAME gaze_following_demo)
include(cmake/pods.cmake)

pods_install_python_script(oh-send-neck-command scripts/send-neck-command.py)

add_subdirectory(src/)
