set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")  
  
add_executable(drc-printf-cache printf-cache.cpp  )
pods_use_pkg_config_packages(drc-printf-cache  lcm)
pods_install_executables(drc-printf-cache )

add_executable(drc-printf-request printf-request.cpp  )
pods_use_pkg_config_packages(drc-printf-request  lcm)
pods_install_executables(drc-printf-request )
