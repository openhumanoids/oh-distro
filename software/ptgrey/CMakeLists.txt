cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME ptgrey)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(ptgrey-gige src/ptgrey-gige.cpp)
pods_install_executables(ptgrey-gige)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wreturn-type -Wuninitialized -Wunused-variable")

set(REQUIRED_PACKAGES
  flycapture
  opencv
  bot2-core
  jpeg-utils
  lcm
)
pods_use_pkg_config_packages(ptgrey-gige ${REQUIRED_PACKAGES})
