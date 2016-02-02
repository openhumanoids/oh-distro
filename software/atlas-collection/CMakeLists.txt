cmake_minimum_required(VERSION 2.8.5)

set(POD_NAME atlas-collection)
include(cmake/pods.cmake)
include(ExternalProject)

set(default_cmake_args
  -DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}
  -DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}
)

# Macro to add external projects with dependencies
macro(add_external_project proj)
  ExternalProject_Add(${proj}
    SOURCE_DIR ${PROJECT_SOURCE_DIR}/${proj}
    BINARY_DIR ${PROJECT_SOURCE_DIR}/${proj}/pod-build
    CMAKE_CACHE_ARGS ${default_cmake_args}
    DEPENDS ${${proj}_depends})
endmacro()


# Only build if private modules atlas and flycapture have been checked out
if(IS_DIRECTORY ${PROJECT_SOURCE_DIR}/atlas)
  add_external_project(atlas_lcmtypes)

  set(atlas_depends atlas_lcmtypes)
  add_external_project(atlas)

  set(state_sync_atlas_depends atlas atlas_lcmtypes)
  add_external_project(state_sync_atlas)

  add_external_project(bdi_walking)

  add_external_project(flycapture)

  set(ptgrey_depends flycapture)
  add_external_project(ptgrey)
else()
  message(WARNING "Atlas does not exist - skip building atlas")
endif()
