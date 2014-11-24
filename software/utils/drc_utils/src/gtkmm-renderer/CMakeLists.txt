find_package(PkgConfig REQUIRED)
add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

#####################################################################

add_library(gtkmm-renderer SHARED 
  RendererBase.cpp
)
target_link_libraries(gtkmm-renderer drc-utils)
  
pods_use_pkg_config_packages(gtkmm-renderer
  bot2-vis gtkmm-2.4)

pods_install_headers(RendererBase.hpp
  DESTINATION gtkmm-renderer)

pods_install_libraries(gtkmm-renderer)
  
pods_install_pkg_config_file(gtkmm-renderer
    LIBS -lgtkmm-renderer
    REQUIRES bot2-vis
    VERSION 0.0.1)
