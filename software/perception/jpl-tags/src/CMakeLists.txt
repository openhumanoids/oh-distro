set(APP_NAME drc-jpl-tags)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall")

# Create an executable program hello-test
add_executable(${APP_NAME} jpl-tags-driver.cpp)

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(${APP_NAME} jpl-tags drc-utils)

# make executable public
pods_install_executables(${APP_NAME})
