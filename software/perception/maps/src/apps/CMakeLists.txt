set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wno-attributes -Wreturn-type -Wuninitialized -Wunused-variable")

add_subdirectory(server)

add_executable(data-request-server DataRequestServer.cpp RobotState.cpp)
pods_use_pkg_config_packages(data-request-server drc-utils  maps) # affordance
pods_install_executables(data-request-server)

add_executable(maps-test TestMain.cpp)
pods_use_pkg_config_packages(maps-test maps bot2-lcmgl-client)
pods_install_executables(maps-test)

add_executable(maps-test-collector TestCollector.cpp)
pods_use_pkg_config_packages(maps-test-collector maps bot2-lcmgl-client)
pods_install_executables(maps-test-collector)

add_executable(maps-snapshot-server SnapshotServer.cpp)
pods_use_pkg_config_packages(maps-snapshot-server maps)
pods_install_executables(maps-snapshot-server)

add_executable(maps-registrar PointCloudRegistrar.cpp)
pods_use_pkg_config_packages(maps-registrar maps bot2-lcmgl-client)
pods_install_executables(maps-registrar)

#add_executable(maps-affordance-updater AffordanceUpdater.cpp)
#pods_use_pkg_config_packages(maps-affordance-updater affordance drc-utils)
#pods_install_executables(maps-affordance-updater)
