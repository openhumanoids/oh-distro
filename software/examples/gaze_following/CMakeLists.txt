cmake_minimum_required(VERSION 2.6.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(POD_NAME gaze_following_demo)
include(cmake/pods.cmake)

add_subdirectory(src/)
