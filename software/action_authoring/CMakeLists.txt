cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME action-authoring)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


set(REQUIRED_PACKAGES libxml-2.0)
add_executable(runme src/Constraint.cpp
  src/AffordanceRelation.cpp
  src/DatabaseManager.cpp
  src/test_database_manager.cpp)

pods_use_pkg_config_packages(runme ${REQUIRED_PACKAGES})

#QT includes
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

#boost includes
find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

qt_wrap_cpp(action-authoring MOC_SRCS src/gui/mainwindow.h src/gui/togglepanel.h src/gui/customslider.h src/gui/Qt4Constraint.h)

set(REQUIRED_PACKAGES affordance opengl_object_gfe state_gfe qt4_widget
	collision_detector opengl_scene opengl_object opengl_camera QtCore QtGui QtOpenGL glu gl urdf)

add_executable(action-authoring-gui src/gui/main.cpp src/gui/mainwindow.cpp 
				    src/AtomicConstraint.cpp
				    src/gui/togglepanel.cpp
				    src/gui/Qt4Constraint.cpp
				    src/opengl/ColorRobot.cpp
				    src/opengl/SelectableOpenGLWidget.cpp
					${MOC_SRCS})

target_link_libraries(action-authoring-gui ${Boost_LIBRARIES})

pods_use_pkg_config_packages(action-authoring-gui ${REQUIRED_PACKAGES})


#demo populate
add_executable(demo_populate_aff_server src/test/demo_populate_affordance_server.cpp)
target_link_libraries(demo_populate_aff_server ${Boost_LIBRARIES} lcm affordance)
