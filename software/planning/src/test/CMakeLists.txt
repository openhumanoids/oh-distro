add_definitions(-ggdb3 -std=c++0x)

add_executable(test-FPP test-FPP.cpp)
target_link_libraries(test-FPP boost_system capabilityMap drawingUtil finalPosePlanner drakeIKoptions drakeRigidBodyConstraint)
pods_use_pkg_config_packages(test-FPP bot2-lcmgl-client 
  capabilityMap lcm)
pods_install_executables(test-FPP)
