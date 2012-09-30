#add_definitions(
#    -ggdb3 
#    -std=gnu99
#    )

cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(hubo-lcm)

add_executable(hubo-lcm hubo-lcm.cpp HuboTranslator.cpp )
pods_use_pkg_config_packages(hubo-lcm lcm glib-2.0 PracticalSocket)
pods_install_executables(hubo-lcm )
