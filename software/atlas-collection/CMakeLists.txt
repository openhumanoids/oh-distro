cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME atlas-collection)
include(cmake/pods.cmake) # need to add

add_subdirectory(atlas_lcmtypes)
add_subdirectory(atlas)
add_subdirectory(state_sync_atlas)
add_subdirectory(bdi_walking)
add_subdirectory(flycapture)
add_subdirectory(ptgrey)
