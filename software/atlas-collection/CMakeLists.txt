cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME atlas-collection)
include(cmake/pods.cmake)

# Only build if private modules atlas and flycapture have been checked out
if(IS_DIRECTORY atlas)
  add_subdirectory(atlas_lcmtypes)
  add_subdirectory(atlas)
  add_subdirectory(state_sync_atlas)
  add_subdirectory(bdi_walking)
  add_subdirectory(flycapture)
  add_subdirectory(ptgrey)
else()
  message(WARNING "Atlas does not exist - skip building atlas")
endif()
