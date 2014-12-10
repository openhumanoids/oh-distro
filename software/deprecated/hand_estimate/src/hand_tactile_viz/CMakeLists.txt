add_definitions(-Wall )




##############################################
add_executable (sandia-tactile-viz tactile_viz.cpp)

find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIRS})
link_directories(${X11_LINK_DIRS})
target_link_libraries (sandia-tactile-viz ${X11_LIBRARIES})

pods_use_pkg_config_packages(sandia-tactile-viz
   lcm  path-util)
pods_install_executables(sandia-tactile-viz)

