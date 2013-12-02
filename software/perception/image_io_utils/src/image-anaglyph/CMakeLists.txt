cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(image-anaglyph)

add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

add_executable(drc-image-anaglyph ImageAnaglyphFuser.cpp)
pods_use_pkg_config_packages(drc-image-anaglyph drc-utils bot2-core opencv)
target_link_libraries(drc-image-anaglyph)
pods_install_executables(drc-image-anaglyph)
