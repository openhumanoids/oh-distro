# Create an executable program maps-test
add_executable(maps-builder-robot BuilderRobotMain.cpp)

# TODO: this is a hack to get it to compile;
# should be picking this up from maps pkg-config
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(maps-builder-robot maps)

# make executable public
pods_install_executables(maps-builder-robot)


add_executable(maps-builder-operator BuilderOperatorMain.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(maps-builder-operator bot2-lcmgl-client maps)
pods_install_executables(maps-builder-operator)
