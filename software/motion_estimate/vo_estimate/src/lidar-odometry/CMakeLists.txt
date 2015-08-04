set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


add_definitions(-Wall )
#add_definitions(-Wall -std=gnu99) # .... compiler warning

add_library(lidar-odom SHARED lidar-odometry.cpp )
pods_use_pkg_config_packages(lidar-odom eigen3 frsm bot2-core)

set_target_properties(lidar-odom PROPERTIES SOVERSION 1)
pods_install_libraries(lidar-odom)
pods_install_headers(lidar-odometry.hpp     DESTINATION lidar-odom)
pods_install_pkg_config_file(lidar-odom
    LIBS -llidar-odom
    REQUIRES 
    VERSION 0.0.1)
    

##############################################
add_executable (se-lidar-odometry lidar-odometry-app.cpp)
target_link_libraries(se-lidar-odometry boost_system)
pods_use_pkg_config_packages(se-lidar-odometry eigen3
  lcm    bot2-frames_cpp
  lcmtypes_bot2-core  
  opencv lidar-odom
  ${FK_PACKAGES})
pods_install_executables(se-lidar-odometry)


##############################################
# Basic Test Program
add_executable (se-lidar-odometry-simple lidar-odometry-app-simple.cpp)
target_link_libraries(se-lidar-odometry-simple boost_system)
pods_use_pkg_config_packages(se-lidar-odometry-simple eigen3
  lcm
  lcmtypes_bot2-core
  opencv lidar-odom
  ${FK_PACKAGES})
pods_install_executables(se-lidar-odometry-simple)
