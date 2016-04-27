cmake_minimum_required(VERSION 2.8.12)

set(POD_NAME oh-examples)
include(cmake/pods.cmake)
include(ExternalProject)

set(default_cmake_args
  -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

# call this macro for each project without dependencies so that it will
# re-check the build in case there are modified files.  In CMake 3.1, you
# can use the BUILD_ALWAYS flag instead, but this is for support of older
# version of cmake.
macro(checkbuild proj)
  ExternalProject_Add_Step(${proj} forceconfigure
      COMMAND ${CMAKE_COMMAND} -E echo "Check build of ${proj}"
      DEPENDEES configure
      DEPENDERS build
      ALWAYS 1)
endmacro()

# Macro to add external projects with dependencies
macro(add_external_project proj)
  ExternalProject_Add(${proj}
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/${proj}
    BINARY_DIR ${PROJECT_SOURCE_DIR}/${proj}/pod-build
    CMAKE_CACHE_ARGS ${default_cmake_args}
    DEPENDS ${${proj}_depends})
endmacro()


add_external_project(gaze_following)
checkbuild(gaze_following)
