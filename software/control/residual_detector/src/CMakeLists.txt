include_directories(${CMAKE_SOURCE_DIR}/src)

add_executable(drc-residual-detector residual-detector.cpp)
target_link_libraries(drc-residual-detector RobotStateDriver FootContactDriver)
pods_use_pkg_config_packages(drc-residual-detector lcmtypes_drc_lcmtypes lcm model-client drake-rbm drake-side eigen3 bot2-param-client)
pods_install_executables(drc-residual-detector)