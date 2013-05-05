find_package(PkgConfig REQUIRED)
add_definitions(-Wall -Wno-unknown-pragmas -Wno-deprecated-declarations
  -Wreturn-type -Wuninitialized -Wunused-variable
  -std=c++0x)

#####################################################################
set(TARGET_NAME tracker-renderer)
add_library(${TARGET_NAME} SHARED
  TrackerRenderer.cpp)
  
set_target_properties(${TARGET_NAME} PROPERTIES SOVERSION 1)

set(REQUIRED_PACKAGES
  drc-utils lcmtypes_drc_lcmtypes gtkmm-renderer gtkmm-2.4 affordance)

pods_use_pkg_config_packages(${TARGET_NAME} ${REQUIRED_PACKAGES})


pods_install_headers(TrackerRenderer.hpp
  DESTINATION ${TARGET_NAME})

pods_install_libraries(${TARGET_NAME})
  
pods_install_pkg_config_file(${TARGET_NAME}
    LIBS -l${TARGET_NAME}
    REQUIRES ${REQUIRED_PACKAGES}
    VERSION 0.0.1)
