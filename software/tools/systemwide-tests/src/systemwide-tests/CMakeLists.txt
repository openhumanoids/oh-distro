set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
 
add_executable (drc-systemwide-tests systemwide-tests.cpp )
pods_use_pkg_config_packages(drc-systemwide-tests   
    pointcloud_math
    eigen3  bot2-core lcmtypes_visualization  lcmtypes_bot2-core)
pods_install_executables(drc-systemwide-tests)



add_executable (drc-pinpoint-fall pinpoint-fall.cpp )
pods_use_pkg_config_packages(drc-pinpoint-fall   
    pointcloud_math
    eigen3  bot2-core lcmtypes_visualization  lcmtypes_bot2-core)
pods_install_executables(drc-pinpoint-fall)