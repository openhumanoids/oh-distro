find_package(PkgConfig REQUIRED)
add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wno-unused-local-typedefs -Wno-attributes
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

set(LIBRARY_NAME data-control-renderer)
add_library(${LIBRARY_NAME} SHARED
  ParamManager.cpp DataControlRenderer.cpp)
  
set_target_properties(${LIBRARY_NAME} PROPERTIES SOVERSION 1)

pods_use_pkg_config_packages(${LIBRARY_NAME}
  affordance drc-utils lcmtypes_drc_lcmtypes gtkmm-renderer gtkmm-2.4)

pods_install_headers(DataControlRenderer.hpp
  DESTINATION ${LIBRARY_NAME})

pods_install_libraries(${LIBRARY_NAME})
  
pods_install_pkg_config_file(${LIBRARY_NAME}
    LIBS -l${LIBRARY_NAME}
    REQUIRES lcmtypes_drc_lcmtypes drc-utils gtkmm-renderer lcmtypes_multisense
    VERSION 0.0.1)
