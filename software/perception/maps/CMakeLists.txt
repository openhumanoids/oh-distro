cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME maps)
include(cmake/pods.cmake)

include(cmake/lcmtypes.cmake)
lcmtypes_build()

#tell cmake to build these subdirectories
add_subdirectory(src)
add_subdirectory(src/apps)
add_subdirectory(src/libs)

add_subdirectory(src/utils)

# Maps GTK renderer works as of Dec 2015
#add_subdirectory(src/gtkmm-renderer)
#add_subdirectory(src/renderers)
