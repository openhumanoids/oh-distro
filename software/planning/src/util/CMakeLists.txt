add_definitions(-ggdb3 -std=c++0x)

add_library(fppUtil  SHARED fppUtil.cpp)
pods_use_pkg_config_packages(fppUtil eigen3 drake-rbm)
set_target_properties(fppUtil PROPERTIES SOVERSION 1)
pods_install_libraries(fppUtil)
pods_install_headers( fppUtil.hpp
              DESTINATION fppUtil)
pods_install_pkg_config_file(fppUtil
  LIBS 
  REQUIRES eigen3
  VERSION 0.0.1)

