set(APP_NAME drc-lcm-bridge)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++1y -Wall")

add_executable(${APP_NAME}
  Bridge.cpp
  Main.cpp
)

pods_use_pkg_config_packages(${APP_NAME} drc-utils)
pods_install_executables(${APP_NAME})
