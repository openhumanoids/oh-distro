add_definitions(-ggdb3 -std=c++0x)

add_library(capabilityMap  SHARED CapabilityMap.cpp)
pods_use_pkg_config_packages(capabilityMap  fppUtil eigen3 drake-rbm)
set_target_properties(capabilityMap PROPERTIES SOVERSION 1)
pods_install_libraries(capabilityMap)
pods_install_headers( CapabilityMap.hpp
              DESTINATION capabilityMap)
pods_install_pkg_config_file(capabilityMap
#  LIBS mt19937ar
  REQUIRES eigen3
  VERSION 0.0.1)

