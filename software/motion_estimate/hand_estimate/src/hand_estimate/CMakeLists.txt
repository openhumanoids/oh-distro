add_definitions(-Wall )




##############################################
add_executable (sandia-process sandia_hand_process.cpp)
target_link_libraries (sandia-process )
pods_use_pkg_config_packages(sandia-process
   lcm )
pods_install_executables(sandia-process)






##############################################
add_executable (sandia-analysis sandia_hand_analysis.cpp)
target_link_libraries (sandia-analysis )
pods_use_pkg_config_packages(sandia-analysis
   lcm )
pods_install_executables(sandia-analysis)
