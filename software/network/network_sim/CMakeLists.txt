cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME network_sim)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake/")

find_package(ProtobufLocal REQUIRED)
include_directories(${PROTOBUF_INCLUDE_DIR})

#set(GOBY_ROOT_DIR "/home/toby/goby/2.0-drc")
find_package(Goby REQUIRED)
include_directories(${GOBY_INCLUDE_DIR})

protobuf_include_dirs(${PROTOBUF_INCLUDE_DIR})
protobuf_include_dirs(${GOBY_INCLUDE_DIR})

# simple network simulator:
add_subdirectory(src/network_bridge)
add_subdirectory(src/log_vrcscore)


