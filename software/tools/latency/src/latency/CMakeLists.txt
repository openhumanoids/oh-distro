set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
################ LIBRARIES ###############################
add_library(latency  SHARED latency.cpp)
#pods_use_pkg_config_packages(latency)
set_target_properties(latency PROPERTIES SOVERSION 1)
pods_install_libraries(latency)
pods_install_headers( latency.hpp DESTINATION latency)
pods_install_pkg_config_file(latency
  LIBS -llatency
  VERSION 0.0.1)

  
  
  
  
add_executable(drc-latency latency-app.cpp  )
pods_use_pkg_config_packages(drc-latency latency
   lcmtypes_drc_lcmtypes lcmtypes_bot2-core lcm)
pods_install_executables(drc-latency )