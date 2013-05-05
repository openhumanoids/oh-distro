set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++0x -Wno-deprecated-declarations -Wreturn-type -Wuninitialized -Wunused-variable")

set(TARGET_NAME "affordance-gaze")
add_executable(${TARGET_NAME} AffordanceGazeMain.cpp)
pods_use_pkg_config_packages(${TARGET_NAME} drc-utils affordance)
pods_install_executables(${TARGET_NAME})
