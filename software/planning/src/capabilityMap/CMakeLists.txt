add_definitions(-ggdb3 -std=c++0x)

add_library(capabilityMap  SHARED capabilityMap.cpp)
pods_use_pkg_config_packages(capabilityMap  eigen3 drake-rbm)
set_target_properties(capabilityMap PROPERTIES SOVERSION 1)
pods_install_libraries(capabilityMap)
pods_install_headers( capabilityMap.hpp
              DESTINATION capabilityMap)
pods_install_pkg_config_file(capabilityMap
  LIBS 
  REQUIRES eigen3
  VERSION 0.0.1)

