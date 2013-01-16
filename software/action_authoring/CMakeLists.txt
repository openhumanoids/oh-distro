cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME action-authoring)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

# build two LCM test programs
#add_executable(send_message src/send_message.c)
#pods_use_pkg_config_packages(send_message lcm lcmtypes_${POD_NAME})

#add_executable(listener src/listener.c)
#pods_use_pkg_config_packages(listener lcm lcmtypes_${POD_NAME})

# make exeutables public
#pods_install_executables(send_message listener)
add_executable(runme src/Constraint.cpp src/Action.cpp src/test.cpp src/Affordance.cpp)

find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

qt_wrap_cpp(action-authoring MOC_SRCS src/gui/mainwindow.h src/gui/togglepanel.h)

set(REQUIRED_PACKAGES opengl_object_gfe state_gfe qt4_widget opengl_scene opengl_object opengl_camera QtCore QtGui QtOpenGL glu gl urdf)

add_executable(action-authoring-gui src/gui/main.cpp src/gui/mainwindow.cpp 
				    src/Affordance.cpp src/Constraint.cpp
				    src/gui/togglepanel.cpp
				    src/opengl/ColorRobot.cpp
				    ${MOC_SRCS})
pods_use_pkg_config_packages(action-authoring-gui ${REQUIRED_PACKAGES})
