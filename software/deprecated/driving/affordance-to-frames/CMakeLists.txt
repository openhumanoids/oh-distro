cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME affordance-to-frames)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(affordance-to-frames src/main.cpp)

pods_use_pkg_config_packages(affordance-to-frames
    lcm 
    glib-2.0 
    bot2-core
    bot2-param-client
    bot2-lcmgl-client
    bot2-frames
    lcmtypes_drc_lcmtypes
)

pods_install_executables(affordance-to-frames)
