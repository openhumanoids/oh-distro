cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME 3d-mouse)
include(cmake/pods.cmake)

# create a library, and make it public
add_library(${POD_NAME} SHARED src/SpaceMouse.cpp)
pods_install_headers(
  src/SpaceMouse.hpp
  DESTINATION ${POD_NAME}
)
pods_install_libraries(${POD_NAME})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wreturn-type -Wuninitialized -Wunused-variable")

#set(REQUIRED_PACKAGES
##none
#)
#
#pods_use_pkg_config_packages(${POD_NAME} ${REQUIRED_PACKAGES})

pods_install_pkg_config_file(${POD_NAME}
  CFLAGS -std=c++0x
  LIBS -l${POD_NAME} -lspnav -lpthread
  REQUIRES ${REQUIRED_PACKAGES}
  VERSION 0.0.1
)
