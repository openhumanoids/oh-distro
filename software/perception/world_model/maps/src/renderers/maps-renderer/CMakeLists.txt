find_package(PkgConfig REQUIRED)
add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable -std=c++0x)

set(LIBRARY_NAME maps-renderer)
add_library(${LIBRARY_NAME} SHARED
  MapsRenderer.cpp
  MeshRenderer.cpp
  InteractiveBox.cpp)
  
set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(${LIBRARY_NAME}
  maps jpeg-utils occ-map gtkmm-renderer gtkmm-2.4)

pods_install_headers(MapsRenderer.hpp
  DESTINATION ${LIBRARY_NAME})

pods_install_libraries(${LIBRARY_NAME})
  
pods_install_pkg_config_file(${LIBRARY_NAME}
    LIBS -l${LIBRARY_NAME}
    REQUIRES bot2-vis maps
    VERSION 0.0.1)
