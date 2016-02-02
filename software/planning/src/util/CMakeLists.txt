add_definitions(-ggdb3 -std=c++0x)

add_library(drawingUtil  SHARED drawingUtil.cpp)
pods_use_pkg_config_packages(drawingUtil eigen3)
set_target_properties(drawingUtil PROPERTIES SOVERSION 1)
pods_install_libraries(drawingUtil)
pods_install_headers( drawingUtil.hpp
              DESTINATION drawingUtil)
pods_install_pkg_config_file(drawingUtil
  LIBS 
  REQUIRES eigen3
  VERSION 0.0.1)

