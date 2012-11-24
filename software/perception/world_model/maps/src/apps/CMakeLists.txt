# Create an executable program maps-test
add_executable(maps-builder-robot BuilderRobotMain.cpp)

# TODO: this is a hack to get it to compile;
# should be picking this up from maps pkg-config
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")

# The test program uses the shared library, use the pkg-config file
pods_use_pkg_config_packages(maps-builder-robot maps)

# make executable public
pods_install_executables(maps-builder-robot)


add_executable(maps-builder-base BuilderBaseMain.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(maps-builder-base bot2-lcmgl-client maps)
pods_install_executables(maps-builder-base)


#add_executable(maps-client-sample SampleMapClient.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-client-sample maps)
#pods_install_executables(maps-client-sample)

add_executable(maps-heightmap-publisher HeightMapPublisher.cpp)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
pods_use_pkg_config_packages(maps-heightmap-publisher bot2-lcmgl-client maps)
pods_install_executables(maps-heightmap-publisher)

#add_executable(maps-params-publisher ParamsPublisher.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-params-publisher maps)
#pods_install_executables(maps-params-publisher)
