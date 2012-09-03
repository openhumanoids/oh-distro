# Create an executable program maps-test
add_executable(maps-test maps-test.cpp)

# TODO: this is a hack to get it to compile;
# should be picking this up from maps pkg-config
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(maps-test maps lcm)

# make executable public
pods_install_executables(maps-test)
