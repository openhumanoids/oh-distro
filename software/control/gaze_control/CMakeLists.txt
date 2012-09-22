cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME gaze_control)
include(cmake/pods.cmake)

FILE(GLOB GAZE_CONTROL_HDRS include/gaze_control/[^.]*.hpp)
pods_install_headers(${GAZE_CONTROL_HDRS} DESTINATION gaze_control)



