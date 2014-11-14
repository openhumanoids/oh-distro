cmake_minimum_required(VERSION 2.6.1)

set(POD_NAME multisense-lib)
include(cmake/pods.cmake)

#copy over/install the shared object
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pkg/lib/libMultiSense.so.3.0 ${CMAKE_BINARY_DIR}/lib/libMultiSense.so.3.0 COPYONLY)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/pkg/lib/libMultiSense.so.3.0
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/pkg/lib/libMultiSense.so ${CMAKE_BINARY_DIR}/lib/libMultiSense.so COPYONLY)
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/pkg/lib/libMultiSense.so
                DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

# ditto for headers
file(GLOB MULTISENSE_HEADERS pkg/include/*.hh)
install(FILES ${MULTISENSE_HEADERS} DESTINATION include/${POD_NAME})

pods_install_pkg_config_file(${POD_NAME}
    LIBS -lMultiSense
    VERSION 2.6.1)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/pkg/include)
