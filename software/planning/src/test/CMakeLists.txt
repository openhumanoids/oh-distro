add_definitions(-ggdb3 -std=c++0x)

add_executable(test-gl test-gl.cpp)
target_link_libraries(test-gl boost_system capabilityMap drawingUtil finalPosePlanner drakeIKoptions)
pods_use_pkg_config_packages(test-gl bot2-lcmgl-client 
  capabilityMap lcm)
pods_install_executables(test-gl)
