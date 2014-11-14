set(DRIVER_NAME multisense-lcm-driver)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

add_executable(${DRIVER_NAME}
  Sensor.cpp
  Imu.cpp
  Laser.cpp
  Camera.cpp
  Driver.cpp
  Main.cpp
)

pods_use_pkg_config_packages(${DRIVER_NAME}
  multisense-lib lcm opencv zlib
)

pods_install_executables(${DRIVER_NAME})
