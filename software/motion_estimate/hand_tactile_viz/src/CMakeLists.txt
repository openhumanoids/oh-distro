add_definitions(-Wall )




##############################################
add_executable (tactile_viz tactile_viz.cpp)

find_package(X11 REQUIRED)
include_directories(${X11_INCLUDE_DIRS})
link_directories(${X11_LINK_DIRS})
target_link_libraries (tactile_viz ${X11_LIBRARIES})
#find_library(X11_LIBRARY NAMES X11
#             HINTS ${PC_LIBXML_LIBDIR} ${PC_LIBXML_LIBRARY_DIRS} )

#set(LIBXML2_LIBRARIES ${LIBXML2_LIBRARY} )
#set(LIBXML2_INCLUDE_DIRS ${LIBXML2_INCLUDE_DIR} )

#include_directories(/usr/X11R6/include)
#LINK_DIRECTORIES(/usr/X11R6/lib)
#target_link_libraries (tactile_viz X11)


pods_use_pkg_config_packages(tactile_viz
   lcm )
pods_install_executables(tactile_viz)

