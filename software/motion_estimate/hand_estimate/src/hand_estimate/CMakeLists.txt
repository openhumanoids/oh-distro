add_definitions(-Wall )




##############################################
add_executable (sandia_hand_process sandia_hand_process.cpp)
target_link_libraries (sandia_hand_process )
pods_use_pkg_config_packages(sandia_hand_process
   lcm )
pods_install_executables(sandia_hand_process)
