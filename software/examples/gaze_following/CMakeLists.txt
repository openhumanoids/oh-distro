cmake_minimum_required(VERSION 2.6.0)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(POD_NAME gaze_following_demo)
include(cmake/pods.cmake)

find_package(PkgConfig REQUIRED)
pkg_check_modules(drake-ik drake-ik)

if(drake-ik_FOUND)
	add_subdirectory(src/)
else()
	message(WARNING "drake-ik not found, skipping gaze following demo")
endif()
