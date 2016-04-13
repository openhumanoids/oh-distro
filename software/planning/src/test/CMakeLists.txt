add_definitions(-ggdb3 -std=c++0x)

add_executable(batch-FPP batch-FPP.cpp)
target_link_libraries(batch-FPP boost_system capabilityMap fppUtil finalPosePlanner drakeIKoptions drakeRigidBodyConstraint tinyxml2 boost_program_options)
pods_use_pkg_config_packages(batch-FPP bot2-lcmgl-client
  capabilityMap lcm)
add_executable(test-FPP test-FPP.cpp)
target_link_libraries(test-FPP capabilityMap fppUtil finalPosePlanner drakeIKoptions drakeRigidBodyConstraint)
pods_use_pkg_config_packages(test-FPP bot2-lcmgl-client
  capabilityMap lcm)
pods_install_executables(batch-FPP test-FPP)
