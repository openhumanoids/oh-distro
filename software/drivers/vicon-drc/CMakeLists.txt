cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME vicon-drc)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()




# build LCM vrt
add_subdirectory(src)
add_subdirectory(plots)

# install the SDK library file and header file:
configure_file(src/libViconDataStreamSDK_CPP.so ${CMAKE_BINARY_DIR}/lib/libViconDataStreamSDK_CPP.so COPYONLY)
install(FILES src/libViconDataStreamSDK_CPP.so 
                DESTINATION ${CMAKE_BINARY_DIR}/lib)
install(FILES src/libViconDataStreamSDK_CPP.so DESTINATION lib)

pods_install_headers(src/Client.h DESTINATION ViconDataStreamSDK_CPP)

