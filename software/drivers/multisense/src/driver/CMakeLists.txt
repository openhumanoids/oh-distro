set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

add_library(multisense_lcm SHARED laser.cpp camera.cpp imu.cpp)

pods_use_pkg_config_packages(multisense_lcm lcm 
  lcmtypes_bot2-core opencv kdl-parser 
  multisense_lib)

set_target_properties(multisense_lcm PROPERTIES SOVERSION 1)
pods_install_headers(camera.h laser.h imu.h DESTINATION multisense_lcm )

pods_install_libraries(multisense_lcm)

target_link_libraries(multisense_lcm z)
pods_install_pkg_config_file(multisense_lcm
    LIBS -lmultisense_lcm -lz -lmultisense_lib
    REQUIRES lcmtypes_bot2-core
    VERSION 0.0.1)

#############################################################3                                     
add_executable(drc-multisense driver.cpp )
target_link_libraries(drc-multisense multisense_lcm )
pods_use_pkg_config_packages(drc-multisense lcm   kdl-parser urdf urdf_dom
                             lcmtypes_bot2-core  opencv lcm multisense_lib
                             lcmtypes_multisense) 
pods_install_executables(drc-multisense)
