cmake_minimum_required(VERSION 2.8.5)

set(POD_NAME atlas-collection)
include(cmake/pods.cmake)
include(ExternalProject)

if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/atlas)
  set(atlas_enabled_default ON)
else()
  set(atlas_enabled_default OFF)
endif()

# Deactivate Atlas driver for Mac
if(APPLE)
  set(atlas_enabled_default OFF)
endif()

option(BUILD_ATLAS_DRIVER "Build the Atlas driver" ${atlas_enabled_default})


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


add_external_project(atlas_lcmtypes)
checkbuild(atlas_lcmtypes)

add_external_project(bdi_walking)
checkbuild(bdi_walking)

# Only build if private modules atlas and flycapture have been checked out
if(BUILD_ATLAS_DRIVER)

  set(atlas_depends atlas_lcmtypes)
  add_external_project(atlas)
  checkbuild(atlas)

  set(state_sync_atlas_depends atlas atlas_lcmtypes)
  add_external_project(state_sync_atlas)
  checkbuild(state_sync_atlas)

  add_external_project(flycapture)
  checkbuild(flycapture)

  set(ptgrey_depends flycapture)
  add_external_project(ptgrey)
  checkbuild(ptgrey)
endif()
