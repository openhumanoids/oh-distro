cmake_minimum_required(VERSION 2.6.1)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME multisense)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

##############################################################################
##############################################################################
set(DIR multisense_lib) 
#copy over/install the shared object
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/lib/libMultiSense.so.2.0 ${CMAKE_BINARY_DIR}/lib/libMultiSense.so.2.0 COPYONLY)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/lib/libMultiSense.so.2.0
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/lib/libMultiSense.so ${CMAKE_BINARY_DIR}/lib/libMultiSense.so COPYONLY)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/lib/libMultiSense.so
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

# ditto for headers
file(GLOB MULTISENSE_HEADERS ${DIR}/include/*.hh)
install(FILES ${MULTISENSE_HEADERS} DESTINATION include/multisense_lib)

pods_install_pkg_config_file(multisense_lib
    LIBS -lMultiSense
    VERSION 2.6.1)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/${DIR}/include)
##############################################################################
##############################################################################
add_subdirectory(src/driver)

add_subdirectory(src/multisense-glview)
add_subdirectory(src/multisense-renderer)

add_subdirectory(src/project-lidar)
