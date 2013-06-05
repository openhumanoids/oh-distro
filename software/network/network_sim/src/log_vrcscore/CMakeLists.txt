set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


add_executable (drc-log-vrcscore log_vrcscore.cpp )
target_link_libraries(drc-log-vrcscore boost_date_time)
pods_use_pkg_config_packages(drc-log-vrcscore   
    pointcloud_math
    eigen3  bot2-core lcmtypes_visualization  lcmtypes_bot2-core )
pods_install_executables(drc-log-vrcscore)
