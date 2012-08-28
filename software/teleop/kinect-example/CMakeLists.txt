cmake_minimum_required(VERSION 2.6.0)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME mycode)
include(cmake/pods.cmake)

# create an executable, and make it public
add_executable(hello src/hello.c)
pods_install_executables(hello)

add_executable(kinect_example src/kinect_example.cpp)
pods_use_pkg_config_packages(kinect_example    lcmtypes_kinect
    glib-2.0     lcm    bot2-core    )
pods_install_executables(kinect_example)

add_executable(example src_cpp_exp/example.cpp)
pods_use_pkg_config_packages(example  
    glib-2.0     lcm    bot2-core    )
pods_install_executables(example)

add_executable(listener src_cpp_exp/lcm_listener)
pods_use_pkg_config_packages(listener  
    glib-2.0     lcm    bot2-core    )
pods_install_executables(listener)

add_executable(talker src_cpp_exp/lcm_talker.cpp)
pods_use_pkg_config_packages(talker  
    glib-2.0     lcm    bot2-core    )
pods_install_executables(talker)
