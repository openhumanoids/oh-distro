include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${GUROBI_HOME}/lib) # super hacky, remove this later

pods_find_pkg_config(gurobi)
add_executable(drc-residual-detector residual-detector.cpp ContactFilter.hpp ContactFilter.cpp)
target_link_libraries(drc-residual-detector RobotStateDriver FootContactDriver libgurobi_c++.a)
pods_use_pkg_config_packages(drc-residual-detector lcmtypes_drc_lcmtypes lcm model-client drake-rbm drake-side eigen3 bot2-param-client gurobi)
pods_install_executables(drc-residual-detector)