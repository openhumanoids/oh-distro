cmake_minimum_required(VERSION 2.8.0)
find_program(ROSPACK_PATH rospack)
if(NOT ROSPACK_PATH)
  message(FATAL_ERROR "Can't find path to rospack executable, which is needed to verify your ROS_PACKAGE_PATH.  Did you source a ROS setup file (e.g., /opt/ros/fuerte/setup.sh)?")
endif()
get_filename_component(CurrentFolder ${CMAKE_CURRENT_LIST_DIR} NAME)
execute_process(COMMAND ${ROSPACK_PATH} deps ${CurrentFolder}
                OUTPUT_VARIABLE rospack_output
                RESULT_VARIABLE rospack_error
                ERROR_VARIABLE rospack_stderr
                OUTPUT_STRIP_TRAILING_WHITESPACE)
if(rospack_stderr)
  message(FATAL_ERROR "You have a ROS package dependency problem: ${rospack_stderr}")
endif()
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
set(ENV{ROS_PACKAGE_PATH} ${PROJECT_SOURCE_DIR}:$ENV{ROS_PACKAGE_PATH})
rosbuild_init()
message("sandia_hand_teleop install prefix: ${CMAKE_INSTALL_PREFIX}")
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(ROS_PKG_INSTALL_SHARE_PATH ${CMAKE_INSTALL_PREFIX}/share/sandia-hand-${SANDIA_HAND_VERSION}/ros/${PROJECT_NAME})

install(FILES manifest.xml 
        DESTINATION ${ROS_PKG_INSTALL_SHARE_PATH})

add_subdirectory(launch)
add_subdirectory(nanokontrol)
add_subdirectory(simple_grasp)
