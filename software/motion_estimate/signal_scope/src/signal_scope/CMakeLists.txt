find_package(Qt4 REQUIRED QtCore QtGui QtScript)
include(${QT_USE_FILE})

find_package(PkgConfig)
pkg_check_modules(DRCLCMTYPES drc-lcmtypes)

link_directories(${DRCLCMTYPES_LIBRARY_DIRS})
include_directories(${DRCLCMTYPES_INCLUDE_DIRS})

find_path(QWT_INCLUDE_DIR qwt.h /usr/include/qwt)
find_library(QWT_LIBRARY qwt)

include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${QWT_INCLUDE_DIR})

if(USE_PYTHON)
  find_package(PythonLibs REQUIRED)
  find_package(PythonQt REQUIRED)
  find_package(CTKPythonConsole REQUIRED)

  include_directories(${PYTHON_INCLUDE_DIRS})
  include_directories(${PYTHONQT_INCLUDE_DIRS})
  include_directories(${CTK_PYTHONCONSOLE_INCLUDE_DIRS})
endif()


qt4_wrap_cpp(moc_srcs
  mainwindow.h
  plot.h
  plotwidget.h
  lcmthread.h
  lcmsubscriber.h
  signalhandler.h
  setscaledialog.h
  selectsignaldialog.h
  )

qt4_wrap_ui(ui_srcs
  mainwindow.ui
  setscaledialog.ui
  selectsignaldialog.ui
  )

set(srcs
  ${moc_srcs}
  ${ui_srcs}
  curvedata.cpp
  jointnames.cpp
  main.cpp
  mainwindow.cpp
  plot.cpp
  plotwidget.cpp
  setscaledialog.cpp
  signaldata.cpp
  signaldescription.cpp
  lcmthread.cpp
  signalhandler.cpp
  selectsignaldialog.cpp
  )

add_executable(signal-scope ${srcs})
target_link_libraries(signal-scope ${QT_LIBRARIES} ${QWT_LIBRARY} ${DRCLCMTYPES_LIBRARIES})


pods_install_headers(
  curvedata.h
  lcmthread.h
  mainwindow.h
  plot.h
  plotwidget.h
  samplingthread.h
  selectsignaldialog.h
  setscaledialog.h
  signaldata.h
  signaldescription.h
  signalhandler.h
  DESTINATION signal_scope)


pods_install_executables(signal-scope)

