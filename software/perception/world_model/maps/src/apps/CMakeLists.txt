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

#add_executable(maps-heightmap-publisher HeightMapPublisher.cpp)
#pods_use_pkg_config_packages(maps-heightmap-publisher maps)
#pods_install_executables(maps-heightmap-publisher)

add_executable(maps-test-collector TestCollector.cpp)
pods_use_pkg_config_packages(maps-test-collector maps bot2-lcmgl-client)
pods_install_executables(maps-test-collector)


#add_executable(maps-builder-robot BuilderRobotMain.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-builder-robot maps)
#pods_install_executables(maps-builder-robot)


#add_executable(maps-builder-base BuilderBaseMain.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-builder-base bot2-lcmgl-client maps)
#pods_install_executables(maps-builder-base)


#add_executable(maps-client-sample SampleMapClient.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-client-sample maps)
#pods_install_executables(maps-client-sample)

#add_executable(maps-heightmap-publisher HeightMapPublisher.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-heightmap-publisher bot2-lcmgl-client maps)
#pods_install_executables(maps-heightmap-publisher)

#add_executable(maps-params-publisher ParamsPublisher.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-params-publisher maps)
#pods_install_executables(maps-params-publisher)

#add_executable(maps-depthmap-publisher DepthMapPublisher.cpp)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")
#pods_use_pkg_config_packages(maps-depthmap-publisher bot2-lcmgl-client maps)
#pods_install_executables(maps-depthmap-publisher)

#add_executable(maps-recent-scan-publisher RecentScanPublisher.cpp)
#pods_use_pkg_config_packages(maps-recent-scan-publisher bot2-lcmgl-client maps pointcloud_vis)
#pods_install_executables(maps-recent-scan-publisher) 
