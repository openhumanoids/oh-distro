cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME reactive_navigation_2d)
include(cmake/pods.cmake)


FILE(GLOB NAV_CONTROL_HDRS include/reactive_navigation_2d/[^.]*.hpp)
pods_install_headers(${NAV_CONTROL_HDRS} DESTINATION reactive_navigation_2d)


#set(REQUIRED_PACKAGES boost)
# create an executable, and make it public
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
#pods_use_pkg_config_packages(keyboard_teleop lcm lcmtypes_drc_lcmtypes ${REQUIRED_PACKAGES})
pods_use_pkg_config_packages(keyboard_teleop lcm lcmtypes_drc_lcmtypes)

pods_install_executables(keyboard_teleop)
target_link_libraries(keyboard_teleop boost_thread)

#find_package(Boost COMPONENTS system filesystem REQUIRED)
##...
#target_link_libraries(mytarget 
#  ${Boost_FILESYSTEM_LIBRARY}
#  ${Boost_SYSTEM_LIBRARY}
#)
