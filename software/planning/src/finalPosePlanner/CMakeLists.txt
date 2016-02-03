add_definitions(-ggdb3 -std=c++0x)

add_library(finalPosePlanner  SHARED finalPosePlanner.cpp)
pods_use_pkg_config_packages(finalPosePlanner  eigen3 drake-rbm)
set_target_properties(finalPosePlanner PROPERTIES SOVERSION 1)
pods_install_libraries(finalPosePlanner)
pods_install_headers( finalPosePlanner.hpp
              DESTINATION finalPosePlanner)
pods_install_pkg_config_file(finalPosePlanner
  LIBS 
  REQUIRES eigen3
  VERSION 0.0.1)

