add_definitions(-std=gnu99)

#split stereo plugin
add_library(disparity MODULE disparity.cpp)
pods_use_pkg_config_packages    (disparity camunits  glib-2.0)
install(TARGETS disparity LIBRARY DESTINATION ${CAMUNITS_PLUGIN_DEST_PATH})
