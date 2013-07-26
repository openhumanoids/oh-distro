cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME ipwebcam)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(ipwebcam src/ipwebcam.cpp)

pods_install_executables(ipwebcam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wreturn-type -Wuninitialized -Wunused-variable")

set(REQUIRED_PACKAGES
  opencv
  bot2-core
  jpeg-utils
  lcm
)
pods_use_pkg_config_packages(ipwebcam ${REQUIRED_PACKAGES})
