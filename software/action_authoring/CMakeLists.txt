cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME action_authoring)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


set(REQUIRED_PACKAGES affordance opengl_object_gfe state_gfe qt4_widget
  collision_detector opengl_scene opengl_object opengl_camera QtCore QtGui QtOpenGL glu gl urdf
  libxml-2.0)

#QT includes
find_package(Qt4 COMPONENTS QtCore QtGui QtOpenGL REQUIRED)
include(${QT_USE_FILE})

#boost includes
find_package(Boost COMPONENTS thread date_time program_options filesystem system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})


#QT wrap
qt_wrap_cpp(action_authoring MOC_SRCS src/qt4_gui/MainWindow.h
			     src/qt4_widgets/TogglePanel.h
			     src/qt4_widgets/CustomSlider.h 
			     src/qt4_gui/Qt4ConstraintMacro.h
			     src/opengl/SelectableOpenGLWidget.h
			     src/qt4_widgets/ExtendedLabel.h)

#========libary action_authoring
add_library(action_authoring SHARED 
  src/objects/ManipulationRelation.cpp
#  src/objects/RelationState.cpp
  src/objects/AtomicConstraint.cpp
  src/objects/ConstraintMacro.cpp
  src/storage/DatabaseManager.cpp)

pods_install_headers(
  src/objects/ManipulationRelation.h 
  src/objects/RelationState.h 
  src/objects/AtomicConstraint.h 
  src/objects/ConstraintMacro.h
  src/storage/DatabaseManager.h
  DESTINATION action_authoring)

pods_install_libraries(action_authoring)
pods_use_pkg_config_packages(action_authoring ${REQUIRED_PACKAGES} affordance)
target_link_libraries(action_authoring ${Boost_LIBRARIES})
pods_install_pkg_config_file(action_authoring 
  LIBS -laction_authoring
  REQUIRES ${REQUIRED_PACKAGES})

#======action authoring gui executable
add_executable(action_authoring_gui 
  src/qt4_gui/Main.cpp
  src/qt4_gui/MainWindow.cpp 
  src/qt4_widgets/TogglePanel.cpp
  src/qt4_widgets/ExtendedLabel.cpp
  src/qt4_gui/Qt4ConstraintMacro.cpp
  src/opengl/ColorRobot.cpp
  src/opengl/SelectableOpenGLWidget.cpp
  ${MOC_SRCS})

target_link_libraries(action_authoring_gui ${Boost_LIBRARIES} action_authoring)

pods_use_pkg_config_packages(action_authoring_gui ${REQUIRED_PACKAGES})

pods_install_executables(action_authoring_gui)

#=====demo populate executable
add_executable(demo_populate_aff_server src/test/demo_populate_affordance_server.cpp)
target_link_libraries(demo_populate_aff_server ${Boost_LIBRARIES} lcm affordance)

#=====action database 
add_executable(test_database_manager
  src/test/test_database_manager.cpp
  src/storage/DatabaseManager.cpp)
target_link_libraries(test_database_manager ${Boost_LIBRARIES} lcm affordance action_authoring)
pods_use_pkg_config_packages(test_database_manager ${REQUIRED_PACKAGES})
