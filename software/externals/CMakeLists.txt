cmake_minimum_required(VERSION 2.8.5)
project(Externals)

# set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()


option(BUILD_PRIVATE_EXTERNALS "Build private externals (requires access to private repos)" ON)
option(USE_SYSTEM_LCM "Use system LCM (otherwise build LCM will be built)" ON)
option(USE_SYSTEM_OPENCV "Use system OpenCV (~2.4.12) (otherwise OpenCV will be built from source)" OFF)
option(USE_SYSTEM_PCL "Use system PCL 1.7 (otherwise PCL will be built from source)" OFF)

include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${Externals_BINARY_DIR})


set(source_prefix ${Externals_SOURCE_DIR})


set(default_cmake_args
  -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

set(empty_command NONE)
set(download_only_args
  CONFIGURE_COMMAND ${empty_command}
  BUILD_COMMAND ${empty_command}
  INSTALL_COMMAND ${empty_command}
)


# qt4
find_package(Qt4 4.8 REQUIRED)
set(qt_args -DQT_QMAKE_EXECUTABLE:PATH=${QT_QMAKE_EXECUTABLE})

# python
if(APPLE)
  find_program(PYTHON_CONFIG_EXECUTABLE python-config)
  if (NOT PYTHON_CONFIG_EXECUTABLE)
    message(SEND_ERROR "python-config executable not found, but python is required.")
  endif()
  # using "python-config --prefix" so that cmake always uses the python that is
  # in the users path, this is a fix for homebrew on Mac:
  # https://github.com/Homebrew/homebrew/issues/25118
  execute_process(COMMAND ${PYTHON_CONFIG_EXECUTABLE} --prefix OUTPUT_VARIABLE python_prefix OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(PYTHON_INCLUDE_DIR "${python_prefix}/include/python2.7" CACHE PATH "Python include directory.")
  set(PYTHON_LIBRARY "${python_prefix}/lib/libpython2.7${CMAKE_SHARED_LIBRARY_SUFFIX}" CACHE FILEPATH "Python library.")
else()
  find_package(PythonLibs 2.7 REQUIRED)
endif()

set(python_args
  -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_INCLUDE_DIR2:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
  )


# eigen
set(EIGEN3_INCLUDE_DIR ${CMAKE_INSTALL_PREFIX}/include/eigen3)
set(eigen_args
  -DEIGEN_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  -DEIGEN_INCLUDE_DIRS:PATH=${EIGEN3_INCLUDE_DIR}
  -DEIGEN3_INCLUDE_DIR:PATH=${EIGEN3_INCLUDE_DIR}
  )

# vtk
set(vtk_homebrew_dir /usr/local/opt/vtk5/lib/vtk-5.10)
if (APPLE AND IS_DIRECTORY ${vtk_homebrew_dir})
  set(vtk_args -DVTK_DIR:PATH=${vtk_homebrew_dir})
endif()


include(cmake/externals.cmake)
