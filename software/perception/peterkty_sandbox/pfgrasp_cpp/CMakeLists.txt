cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
project(pfgrasp)
set(POD_NAME pfgrasp)
set(PACKAGE_NAME pfgrasp)
include(cmake/pods.cmake)

add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


add_executable(drc-pfgrasp src/pfgrasp.cpp src/ImageWarper.cpp)
pods_use_pkg_config_packages(drc-pfgrasp drc-utils bot2-core opencv bot2-frames tld-tracker)
target_link_libraries(drc-pfgrasp)
pods_install_executables(drc-pfgrasp)

