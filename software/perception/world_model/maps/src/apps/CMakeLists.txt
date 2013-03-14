set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations")

add_executable(maps-server ServerMain.cpp)
pods_use_pkg_config_packages(maps-server maps)
pods_install_executables(maps-server)

add_executable(data-request-server DataRequestServer.cpp)
pods_use_pkg_config_packages(data-request-server drc-utils)
pods_install_executables(data-request-server)

add_executable(maps-test TestMain.cpp)
pods_use_pkg_config_packages(maps-test maps bot2-lcmgl-client)
pods_install_executables(maps-test)

add_executable(maps-test-collector TestCollector.cpp)
pods_use_pkg_config_packages(maps-test-collector maps bot2-lcmgl-client)
pods_install_executables(maps-test-collector)
