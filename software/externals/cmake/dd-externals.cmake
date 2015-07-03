

# Set up default arguments for all projects
option(BUILD_SHARED_LIBS "Build libraries as SHARED" ON)
option(BUILD_DOCUMENTATION "Build documentation (Doxygen)" OFF)
option(ENABLE_TESTING "Enable testing for projects" OFF)


set(DRCExternals_INSTALL_PREFIX "${DRCExternals_SOURCE_DIR}/../build")
set(DRCExternals_DEFAULT_ARGS
  "-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}"
  "-DCMAKE_PREFIX_PATH:PATH=${DRCExternals_INSTALL_PREFIX}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${DRCExternals_INSTALL_PREFIX}"
  "-DBUILD_DOCUMENTATION:BOOL=${BUILD_DOCUMENTATION}"
  "-DENABLE_TESTING:BOOL=${ENABLE_TESTING}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  )


if(NOT DEFINED CMAKE_PREFIX_PATH)
  set(CMAKE_PREFIX_PATH ${DRCExternals_INSTALL_PREFIX} CACHE PATH "Prefix paths that cmake uses to look for lib and include paths")
endif()

# Find required external dependencies
find_package(Qt4 4.8 REQUIRED)
find_package(PythonInterp REQUIRED)
find_package(PythonLibs REQUIRED)
set(EIGEN3_INCLUDE_DIR ${DRCExternals_INSTALL_PREFIX}/include/eigen3)

list(APPEND DRCExternals_DEFAULT_ARGS
  -DQT_QMAKE_EXECUTABLE:PATH=${QT_QMAKE_EXECUTABLE}
  -DPYTHON_EXECUTABLE:PATH=${PYTHON_EXECUTABLE}
  -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_INCLUDE_DIR2:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
  -DEIGEN_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  -DEIGEN_INCLUDE_DIRS:PATH=${EIGEN3_INCLUDE_DIR}
  -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  )



# PythonQt
ExternalProject_Add(PythonQt
  GIT_REPOSITORY https://github.com/commontk/PythonQt.git
  GIT_TAG 00e6c6b2
  CMAKE_CACHE_ARGS
    ${DRCExternals_DEFAULT_ARGS}
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DPythonQt_Wrap_Qtcore:BOOL=ON
    -DPythonQt_Wrap_Qtgui:BOOL=ON
    -DPythonQt_Wrap_Qtuitools:BOOL=ON
    ${DRCExternals_THIRDPARTYLIBS_ARGS}
  )

# ctkPythonConsole
ExternalProject_Add(ctkPythonConsole
  GIT_REPOSITORY git://github.com/patmarion/ctkPythonConsole
  GIT_TAG b24d917ad
  CMAKE_CACHE_ARGS
    ${DRCExternals_DEFAULT_ARGS}
    ${DRCExternals_THIRDPARTYLIBS_ARGS}
  DEPENDS
    PythonQt
  )

# QtPropertyBrowser
ExternalProject_Add(QtPropertyBrowser
  GIT_REPOSITORY https://github.com/patmarion/QtPropertyBrowser
  GIT_TAG origin/master
  CMAKE_CACHE_ARGS
    ${DRCExternals_DEFAULT_ARGS}
    ${DRCExternals_THIRDPARTYLIBS_ARGS}
  )



# VTK
set(use_system_vtk_default ON)
if(APPLE)
  set(use_system_vtk_default OFF)
endif()

option(USE_SYSTEM_VTK "Use system version of VTK.  If off, VTK will be built." ${use_system_vtk_default})

if(NOT USE_SYSTEM_VTK)
  ExternalProject_Add(vtk
    GIT_REPOSITORY git://vtk.org/VTK.git
    GIT_TAG v5.10.1
    CMAKE_CACHE_ARGS
      ${DRCExternals_DEFAULT_ARGS}
      -DBUILD_SHARED_LIBS:BOOL=ON
      -DBUILD_TESTING:BOOL=OFF
      -DBUILD_EXAMPLES:BOOL=OFF
      -DVTK_USE_GUISUPPORT:BOOL=ON
      -DVTK_USE_QT:BOOL=ON
      -DVTK_WRAP_PYTHON:BOOL=ON
      -DVTK_WRAP_TCL:BOOL=OFF
      -DVTK_USE_TK:BOOL=OFF
      ${DRCExternals_THIRDPARTYLIBS_ARGS}
    )

  set(vtk_args -DVTK_DIR:PATH=${DRCExternals_INSTALL_PREFIX}/lib/vtk-5.10)
  set(vtk_depends vtk)
endif()


# PointCloudLibraryPlugin
ExternalProject_Add(PointCloudLibraryPlugin
  GIT_REPOSITORY https://github.com/patmarion/PointCloudLibraryPlugin.git
  GIT_TAG 8599fd0
  CMAKE_CACHE_ARGS
    ${DRCExternals_DEFAULT_ARGS}
    ${DRCExternals_THIRDPARTYLIBS_ARGS}
    ${vtk_args}
    -DPCL_REQUIRED_VERSION:STRING=1.7
  DEPENDS
    pcl
    ${vtk_depends}
  )


#SOURCE_DIR ${DRCExternals_SOURCE_DIR}/${proj}
#BINARY_DIR ${DRCExternals_SOURCE_DIR}/${proj}/build
#ExternalProject_Add_Step(${proj} make_build_dir
#  COMMAND ${CMAKE_COMMAND} -E make_directory ${DRCExternals_SOURCE_DIR}/${proj}/build
#  DEPENDEES download
#  DEPENDERS configure)

