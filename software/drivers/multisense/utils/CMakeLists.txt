cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME utils)
include(cmake/pods.cmake)

add_subdirectory(src/multisense-glview)
add_subdirectory(src/multisense-renderer)
add_subdirectory(src/project-lidar)
add_subdirectory(src/multisense_image_utils)
add_subdirectory(src/multisense_utils)
add_subdirectory(src/test)
