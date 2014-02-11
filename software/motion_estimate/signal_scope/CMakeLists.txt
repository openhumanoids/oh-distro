cmake_minimum_required(VERSION 2.6.0)

set(POD_NAME signal_scope)
include(cmake/pods.cmake)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_INSTALL_PREFIX}")

option(USE_PYTHON "Enable Python support." ON)

add_subdirectory(src/signal_scope)

