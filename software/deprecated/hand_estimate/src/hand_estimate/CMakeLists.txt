add_definitions(-Wall )




##############################################
add_executable (hand-tactile-process hand_tactile_process.cpp)
target_link_libraries (hand-tactile-process )
pods_use_pkg_config_packages(hand-tactile-process lcm )
pods_install_executables(hand-tactile-process)






##############################################
add_executable (sandia-analysis sandia_hand_analysis.cpp)
target_link_libraries (sandia-analysis )
pods_use_pkg_config_packages(sandia-analysis
   lcm )
pods_install_executables(sandia-analysis)
