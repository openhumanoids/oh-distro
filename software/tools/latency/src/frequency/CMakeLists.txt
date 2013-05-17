set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
################ LIBRARIES ###############################
add_library(frequency  SHARED frequency.cpp)
#pods_use_pkg_config_packages(frequency)
set_target_properties(frequency PROPERTIES SOVERSION 1)
pods_install_libraries(frequency)
pods_install_headers( frequency.hpp DESTINATION frequency)
pods_install_pkg_config_file(frequency
  LIBS -lfrequency
  VERSION 0.0.1)

  
  
  
  
add_executable(drc-frequency frequency-app.cpp  )
pods_use_pkg_config_packages(drc-frequency frequency
             bot2-param-client path-util
             lcmtypes_drc_lcmtypes lcmtypes_bot2-core lcm)
pods_install_executables(drc-frequency )
