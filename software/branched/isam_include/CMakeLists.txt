#main
cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME isam)
include(cmake/pods.cmake)

install(DIRECTORY isam DESTINATION include)
