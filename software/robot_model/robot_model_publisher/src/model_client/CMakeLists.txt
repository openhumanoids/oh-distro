################
add_library(model-client  SHARED model-client.cpp)
pods_use_pkg_config_packages(model-client lcmtypes_drc_lcmtypes
  urdf orocos-kdl kdl-parser forward-kinematics)


target_link_libraries (model-client )
set_target_properties(model-client PROPERTIES SOVERSION 1)
pods_install_libraries(model-client)
pods_install_headers(model-client.hpp DESTINATION model-client)
pods_install_pkg_config_file(model-client
  LIBS -lmodel-client
  VERSION 0.0.1)
