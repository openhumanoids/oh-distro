set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wno-deprecated-declarations")


add_executable(kinect-translate kinect_translate.cpp)
target_link_libraries(kinect-translate)
pods_use_pkg_config_packages(kinect-translate
    lcm)
pods_install_executables(kinect-translate)
