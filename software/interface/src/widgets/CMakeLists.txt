cmake_minimum_required(VERSION 2.6.0)

find_package(Boost COMPONENTS signals REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

# pull in the pods macros. See cmake/pods.cmake for documentation
#set(POD_NAME gui)
#include(cmake/pods.cmake)
#target_link_libraries(boost boost_signal)

add_library(gui_widgets SHARED
GlDrawingArea.cc
AffordanceToolPalette.cc
AffordanceDrawingArea.cc
AffordancePane.cc
MainDrawingArea.cc
MainPane.cc
RobotStateListener.cc
markerwidget.cc
twostatescale.cc
)

# make the header public
# install it to include/urdf_interface
pods_install_libraries(gui_widgets)
pods_install_headers(
GlDrawingArea.h
AffordanceDrawingArea.h
AffordanceToolPalette.h
MainDrawingArea.h
RobotStateListener.h
AffordancePane.h
MainPane.h
markerwidget.h
twostatescale.h DESTINATION gui_widgets)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
#include(cmake/lcmtypes.cmake)
#lcmtypes_build()

# uncomment these lines to link against another library via pkg-config
#set(REQUIRED_PACKAGES gtkglextmm-1.2)
#pods_use_pkg_config_packages(urdf ${REQUIRED_PACKAGES})

# add_subdirectory(src/perception_example)

# build two LCM test programs
#add_executable(main src/main.cc)
#pods_use_pkg_config_packages(main gtkglextmm-1.2 lcm lcmtypes_${POD_NAME})

# make exeutables public
#pods_install_executables(main)

target_link_libraries(gui_widgets 
  ${Boost_SIGNALS_LIBRARY}
)

#pods_install_pkg_config_file(gui_widgets CFLAGS -lgtkgextmm-1.2 LIBS -lgui_widgets -lgtkglextmm-1.2)
pods_install_pkg_config_file(gui_widgets LIBS -lgui_widgets REQUIRES gtkglextmm-1.2)

pods_use_pkg_config_packages(gui_widgets lcmtypes_drc_lcmtypes gtkglextmm-1.2)
