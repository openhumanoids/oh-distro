set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")  
  
add_executable(drc-control-hangup control-hangup.cpp  )
pods_use_pkg_config_packages(drc-control-hangup  lcm)
pods_install_executables(drc-control-hangup )

