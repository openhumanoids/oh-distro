set(LIBRARY_NAME maps-server)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

add_executable(${LIBRARY_NAME}
  LidarFilters.cpp
  FusedDepthHandler.cpp
  StereoHandler.cpp
  ServerMain.cpp)
pods_use_pkg_config_packages(${LIBRARY_NAME} maps opencv)
pods_install_executables(${LIBRARY_NAME})
