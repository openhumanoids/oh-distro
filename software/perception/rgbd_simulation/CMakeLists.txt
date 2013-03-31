cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME rgbd_simulation)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

add_subdirectory(src/rgbd_primitives)

add_subdirectory(src/rgbd_simulation)
add_subdirectory(src/tools)


### Instal frag and vert files where the binary is:
file(COPY gsl_shaders/compute_score.frag DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY gsl_shaders/compute_score.vert DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

file(COPY gsl_shaders/shader.frag DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY gsl_shaders/shader.vert DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

file(COPY gsl_shaders/single_texture.frag DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY gsl_shaders/single_texture.vert DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

file(COPY gsl_shaders/sum_score.frag DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
file(COPY gsl_shaders/sum_score.vert DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
